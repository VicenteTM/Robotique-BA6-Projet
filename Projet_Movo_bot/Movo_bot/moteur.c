#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include <math.h>
#include <motors.h>

#include <moteur.h>
#include <send_receive.h>
#include <capteur.h>
#include <accelerometre.h>

enum {X,Y,M_X,M_Y};     //enum of the 4 directions in anti-clockwise order to indicate to the computer the robot's direction and to calculate the coordinates

//static global
static int16_t coord_x_av;     //used to memorize the previous value of the x coordinate
static int16_t coord_y_av;     //used to memorize the previous value of the y coordinate
static int16_t sensor_threshold = 1000;     //threshold at which an impact is considered
static int16_t error_threshold = 850;       //sensor goal distance value for the robot to be correctely adjusted
//thread
static THD_WORKING_AREA(waMoteur, 1024);
static THD_FUNCTION(Moteur, arg)    //this thread permits commands and states handling and makes the wheels move accordingly, it also handles the data we send to the computer 
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint8_t send = true;
    uint8_t stop_command = false; //if set to true, blocks the command reading
    uint8_t calibration_done = false;  //indicates if the calibration has been done (if we do it twice in a row, our graph becomes false)
    uint8_t threshold_mesured = false;  //indicates if we have measured the sensor threshold to consider an impact
    uint8_t adjust_mesured = false;     //indicates if we have measured the sensor value to indicate a correct adjustment
    uint16_t command;           //value of the command
    uint16_t compteur=0;        //this counter is used to plot the calibration data and to move forward and backward
    uint8_t imu=false;         //variable indicating if we are using the imu
    int16_t pos_l_av=0;         //used to memorize the previous value of the left motor
    int16_t pos_r_av=0;         //used to memorize the previous value of the right motor
    int16_t last_pos=0;         //used to memorize the last position of each motor
    static int direction=X;     //used to memorize the present direction of the robot
    int distance=0;             //used to memorize the distance the robot has moved after a command
    coord_x_av = 0;             //initialize the coordinates
    coord_y_av = 0;             //initialize the coordinates
	while(1)
    {
        wait_send_to_epuck();       //wait for a signal from send_receive that the robot received an instruction
        switch(get_state())     //treat the state desired from the computer
        {
            case IDLE:          //neutral case
                imu = false;     //we aren't going to use the imu
                stop_command = true;    //blocks the thread from treating a command as there isn't one
                calibration_done = false;   //enable a new calibration if neccessary
                left_motor_set_speed(0);
                right_motor_set_speed(0);
                break;
            case CONTROLANDREAD:
                wait_accelerometre_mesure();    //wait until the accelerometer value has been refreshed
                stop_command = false;   //the thread has to treat the command
                calibration_done = false;   //enable a new calibration if neccessary
                break;
            case CALIBRATION:
                imu = false;     //we aren't going to use the imu
                if (!calibration_done) //in order to avoid a double calibration resulting in false conversion values, we allow only once at a time calibration
                {
                    uint16_t data_calibration[2];   //array we send to the computer
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    left_motor_set_speed(-mm_to_step(SPEED)/2); //we go slower to have a better precision
                    right_motor_set_speed(-mm_to_step(SPEED)/2);
                    while (right_motor_get_pos()>(pos_r_av-(5*mm_to_step(DISTANCE_ONE)))) //calibrate on 50 mm
                    {
                        calibrate();    //get the intensity of the two front proximity sensors
                        data_calibration[0] = compteur; //x value to plot on the graph corresponding to the distance the robot has moved
                        data_calibration[1] = (get_capteur_values_to_send()[0]+get_capteur_values_to_send()[1])/2;   //y value to plot corresponding to the average intensity of the 2 front proximity sensors
                        SendUint16ToComputer((BaseSequentialStream *) &SD3, data_calibration, 2);   //send the 2 values to the computer
                        compteur++;
                        if (!threshold_mesured)
                        {
                            if (compteur >= SENSOR_CALIBRATION_DISTANCE)
                            {
                                sensor_threshold = data_calibration[1];
                                threshold_mesured = true;
                            }  
                        }
                        if (!adjust_mesured)
                        {
                            if (compteur >= ERROR_CALIBRATION_DISTANCE)
                            {
                                error_threshold = data_calibration[1];
                                adjust_mesured = true;
                            }  
                        }
                    }
                    left_motor_set_speed(0);    //stop the motors when calibration is done
                    right_motor_set_speed(0);
                    distance=0;     //make sure the robot doesn't get wrong coordinates
                    compteur=0;     //reset variable
                    stop_command=true;   //blocks the thread from treating a command as there isn't one
                    calibration_done = true;
                }
                break;
            case LIVEIMU:
                calibration_done = false; //enable a new calibration if neccessary
                imu = true;     //we are going to use the imu
                stop_command = false;   //when we plot the live IMU, the commands are still active
                break;
        }
        if (!stop_command)  //if the commands aren't blocked
        {
            if (get_impact())    //if an impact is detected
            {
                if ((get_capteur_values_to_send()[3]>sensor_threshold) && (get_capteur_values_to_send()[0]<sensor_threshold))   //check if the robot has turned a bit to the right but is not facing a wall
                    command = ADJUST_RIGHT;   //change the command received by the computed to the emergency protocol
                else
                {
                    if ((get_capteur_values_to_send()[2]>sensor_threshold) && (get_capteur_values_to_send()[1]<sensor_threshold))   //check if the robot has turned a bit to the left but is not facing a wall
                        command = ADJUST_LEFT;   //change the command received by the computed to the emergency protocol
                    else
                    {
                        if (((get_capteur_values_to_send()[0]+get_capteur_values_to_send()[1])/2)>sensor_threshold) //check if the robot has run into a wall
                            command = TURNAROUND;   //change the command received by the computed to the emergency protocol
                        else
                        {
                            command = get_command();    //an impact can be detected for the bumps on the circuit which must be ignored
                            reset_impact();		//reset the value to indicate that the impact has been treated successfully
                        }
                    }
                }
            }
            else
                command = get_command();    //get the command from the value sent by the computer
            switch(command)         //treat the command sent by the computer
            {
                case FORWARD:   //move 40mm forward
                    direction=set_direction(FORWARD,direction);     //set the new direction of the robot
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    last_pos=right_motor_get_pos();     //memorize the position of one wheel for the condition
                    left_motor_set_speed(mm_to_step(SPEED));    //the left wheel will move the same distance as the right one
                    right_motor_set_speed(mm_to_step(SPEED));
                    compteur=0;
                    while (right_motor_get_pos()<(last_pos+(4*mm_to_step(DISTANCE_ONE))))   //for the same wheel, we wait until it has moved 40mm
                    {
                        distance = right_motor_get_pos()-pos_r_av;      //calculate the new distance reached by the robot
                        distance = step_to_mm(distance);    //convert distance in mm
                        if (distance > REFRESH_DISTANCE)   //we send data to the computer every 10mm
                        {
							coord_x_av += set_x(distance,direction);    //calculate the new x coordinate
							coord_y_av += set_y(distance,direction);    //calculate the new Y coordinate
							send_data(coord_x_av,coord_y_av,direction,imu);	//send data to the computer
							pos_r_av = right_motor_get_pos();	//refresh the last position of the robot
                        }
                    }
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    //distance = right_motor_get_pos()-pos_r_av;      //calculate the new distance reached by the robot
                    send = false;
                    break;
                case BACKWARD:  //move 40mm backward
                    direction=set_direction(BACKWARD,direction);     //set the new direction of the robot
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    last_pos=right_motor_get_pos();     //memorize the position of for the condition
                    left_motor_set_speed(-mm_to_step(SPEED));	    //the left wheel will move the same distance as the right one
                    right_motor_set_speed(-mm_to_step(SPEED));
                    compteur = 0;
                    while (right_motor_get_pos()>(last_pos-(2*mm_to_step(DISTANCE_ONE))))   //for the same wheel, we wait until it has moved 40mm
                    {
                    	distance = right_motor_get_pos()-pos_r_av;      //calculate the new distance reached by the robot
						distance = step_to_mm(distance);    //convert distance in mm
						if (distance < -REFRESH_DISTANCE)   //we send data to the computer every 10mm
						{
							coord_x_av += set_x(distance,direction);    //calculate the new x coordinate
							coord_y_av += set_y(distance,direction);    //calculate the new Y coordinate
							send_data(coord_x_av,coord_y_av,direction,imu);	//send data to the computer
							pos_r_av = right_motor_get_pos();	//refresh the last position of the robot
						}
                    }
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    //distance = right_motor_get_pos()-pos_r_av;      //calculate the new distance reached by the robot
                    send = false;
                    break;
                case LEFT:  //turn 90 degrees to the left
                    direction=set_direction(LEFT,direction);     //set the new direction of the robot
                    distance=0;     //the robot has turned on himself, it didn't change coordinates
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(NB_COUNTER_QUARTER)))     //for the same wheel, we wait until it has turned 90 degrees
                    {
                    left_motor_set_speed(-mm_to_step(SPEED)*2);   //if the right wheel is moving forward, the left one must move backward in order to turn
                    right_motor_set_speed(mm_to_step(SPEED)*2); 
                    }
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    send = true;
                    break;
                case ADJUST_LEFT:  //turn to the left until the robot is realigned
                    direction=set_direction(FORWARD,direction); 	//no need to change the direction as this command is used to realign the robot
                    left_motor_set_speed(-mm_to_step(SPEED)/2);		//if the right wheel is moving forward, the left one must move backward in order to turn
                    right_motor_set_speed(mm_to_step(SPEED)/2);		//
                    while ((get_capteur_values_to_send()[4]-get_capteur_values_to_send()[2])<error_threshold)     //we wait until the right sensor is closer to the wall than the front right to realign the robot
                    {
                        wait_capteur_received();		//wait for the sensors values to be refreshed
                    }
                    left_motor_set_speed(0);     //reset the speed of the robot
                    right_motor_set_speed(0);    //reset the speed of the robot
                    distance=-mm_to_step(DISTANCE_ONE);     //the robot has ran into a wall, the wheels have turned in the air
                    reset_impact();		//reset the value to indicate that the impact has been treated successfully
                    send = true;
                    break;
                case RIGHT:     //turn 90 degrees to the right
                    direction=set_direction(RIGHT,direction);    //set the new direction of the robot
                    distance=0;     //the robot has turned on himself, it didn't change coordinates
                    pos_l_av=left_motor_get_pos();      //memorize the position of one wheel to calculate the distance
                    while (left_motor_get_pos()<(pos_l_av+mm_to_step(NB_COUNTER_QUARTER)))  //for the same wheel, we wait until it has turned 90 degrees
                    {
                        left_motor_set_speed(mm_to_step(SPEED)*2);
                        right_motor_set_speed(-mm_to_step(SPEED)*2);      //if the left wheel is moving forward, the right one must move backward in order to turn
                    }
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    send = true;
                    break;
                case ADJUST_RIGHT:    //turn to the right until the robot is realigned
                    direction=set_direction(FORWARD,direction); //no need to change the direction as this command is used to realign the robot
                    left_motor_set_speed(mm_to_step(SPEED)/2);		//if the right wheel is moving forward, the left one must move backward in order to turn
                    right_motor_set_speed(-mm_to_step(SPEED)/2);	//
                    while ((get_capteur_values_to_send()[5]-get_capteur_values_to_send()[3])<error_threshold)     //we wait until the right sensor is closer to the wall than the front right to realign the robot
                    {
                        wait_capteur_received();		//wait for the sensors values to be refreshed
                    }
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    distance=-mm_to_step(DISTANCE_ONE);     //the robot has ran into a wall, the wheels have turned in the air
                    reset_impact();		//reset the value to indicate that the impact has been treated successfully
                    send = true;
                    break;
                case NEUTRE:    //the robot stops
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    distance=0;
                    calibration_done = false;
                    send = true;
                    break;
                case TURNAROUND:    //in case of impact, turn 180 degrees
                    direction=set_direction(TURNAROUND,direction);   //set the new direction of the robot
                    distance=-mm_to_step(2*DISTANCE_ONE);     //the robot has ran into a wall, the wheels have turned in the air
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(NB_COUNTER_HALF)))    //for the same wheel, we wait until it has turned 180 degrees
                    {
                        left_motor_set_speed(-mm_to_step(SPEED)*2);   //if the right wheel is moving forward, the left one must move backward in order to turn
                        right_motor_set_speed(mm_to_step(SPEED)*2);
                    }
                    left_motor_set_speed(0);    //reset the speed of the robot
                    right_motor_set_speed(0);   //reset the speed of the robot
                    reset_impact();		//reset the value to indicate that the impact has been treated successfully
                    send = true;
                    break;
            }
            if (send){
                distance = step_to_mm(distance);    //convert distance in mm
                coord_x_av += set_x(distance,direction);    //calculate the new x coordinate
                coord_y_av += set_y(distance,direction);    //calculate the new Y coordinate
                send_data(coord_x_av,coord_y_av,direction,imu); //send the data to the computer
            }
           
        }
    //chThdSleepMilliseconds(100);	 //waits 0.1 second
    }
}

int set_direction(int move,int direction)
{
	int16_t direction_f=0;
    switch (move)
    {
    	case FORWARD:
    		direction_f = direction;    //the robot must not change direction
    		break;
        case RIGHT:
        	direction_f = (direction-1) % 4;    //the robot must go one direction less because we set the directions in anti-clockwise order
        	if (direction_f == -1)      //sometimes the % makes a mistake and doesn't work for the -1 this is here to adjust this error
        		direction_f = M_Y;
        	break;
        case LEFT:
        	direction_f = (direction+1) % 4;    //the robot must go one direction more because we set the directions in anti-clockwise order
        	break;
        case BACKWARD:
        	direction_f = direction;    //the robot must not change direction the distance will be negative
        	break;
        case TURNAROUND:
        	direction_f = (direction+2) % 4;    //the robot must go two directions more (or less) because we set the directions in anti-clockwise order
        	break;
    }
    return direction_f;
}

uint16_t set_x(uint16_t distance,uint16_t direction)
{
    uint16_t new_x=0;
    switch(direction)
    {
        case X:
            new_x=distance;     //if the robot is moving in the X direction, we have to add the new x value to the previous coordinate
            break;
        case M_X:
            new_x= -distance;   //if the robot is moving in the -X direction, we have to substract the new x value from the previous coordinate
            break;
        case Y:
            new_x=0;    //if the robot is moving in the y direction, no changes in x direction
            break;
        case M_Y:
            new_x= 0;    //if the robot is moving in the y direction, no changes in x direction
            break;
    }
    return new_x;
}

uint16_t set_y(uint16_t distance,uint16_t direction)
{
    uint16_t new_y=0;
    switch(direction)
    {
        case X:
            new_y=0;    //if the robot is moving in the x direction, no changes in y direction
            break;
        case M_X:
            new_y= 0;    //if the robot is moving in the x direction, no changes in y direction
            break;
        case Y:
            new_y = distance;     //if the robot is moving in the Y direction, we have to add the new y value to the previous coordinate
            break;
        case M_Y:
            new_y = -distance;   //if the robot is moving in the -Y direction, we have to substract the new y value from the previous coordinate
            break;
    }
    return new_y;
}

void start_moteur(void)
{
    chThdCreateStatic(waMoteur, sizeof(waMoteur), NORMALPRIO+2, Moteur, NULL);  //creation of the Moteur thread 
}

int16_t mm_to_step(int16_t value_mm)
{
	double value;
    int16_t value_step;
    value =(double) value_mm * NSTEP_ONE_TURN / WHEEL_PERIMETER;    //equation to convert mm value to step value in double format for precision
    value_step=round(value);  //the values we use are int or uint, we need to round the double value to the upper because the wheels may glide
    return value_step;
}

int16_t step_to_mm(int16_t value_step)
{
    double value;
    int16_t value_mm;
    value = (double) value_step * WHEEL_PERIMETER / NSTEP_ONE_TURN;    //equation to convert value value to mm value in double format for precision
    if (value>0)
    	value_mm=round(value);  //the values we use are int or uint, we need to round the double value to the upper because the wheels may glide
    else
    {
    	if (value<0)
    		value_mm=round(value);  //the values we use are int or uint, we need to round the double value to down because the wheels may glide backward
    	else
    		value_mm = 0;
    }
    return value_mm;
}

void send_data(int16_t coord_x,int16_t coord_y, int direction, uint8_t imu)
{
    static uint16_t data[12];   //array we send to the computer
    wait_capteur_received();    //wait for the Capteur thread to finish measuring the intensities
    data[0] = coord_x;                       	//put the values we want to send to the computer in the data array
    data[1] = coord_y;                       	//
    data[2] = direction;                        //
    data[3] = get_capteur_values_to_send()[0];  //
    data[4] = get_capteur_values_to_send()[1];  //
    data[5] = get_capteur_values_to_send()[2];  //
    data[6] = get_capteur_values_to_send()[3];  //
    data[7] = get_capteur_values_to_send()[4];  //
    data[8] = get_capteur_values_to_send()[5];  //
    data[9] = get_capteur_values_to_send()[6];  //
    data[10] = get_capteur_values_to_send()[7]; //
    if (imu)    //if we are using the imu
    {
        data[11]= get_acceleration_y();    //send the data including the acceleration in y direction
        SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 12);  //
        imu = false;    //reset variable
    }
    else
        SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 11);  //send the data to the computer

}

