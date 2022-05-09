#include <stdlib.h>

#include "ch.h"
#include "hal.h"
#include <math.h>
//#include <chprintf.h>
#include <motors.h>
#include <moteur.h>
#include <send_receive.h>
#include <capteur.h>
#include <accelerometre.h>

enum {X,Y,M_X,M_Y};     //enum of the 4 directions in anti-clockwise order to indicate to the computer the robot's direction and to calculate the coordinates


//thread
static THD_WORKING_AREA(waMoteur, 1024);
static THD_FUNCTION(Moteur, arg)    //this thread permits commands and states handling and makes the wheels move accordingly, it also handles the data we send to the computer 
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t send = true; //variable used to indicate if we want to send the data to the computer (mostly to make sure a command has been treated)
    uint16_t stop_command = false; //if set to true, blocks the command reading
    uint16_t calibration_done = false;  //indicates if the calibration has been done (if we do it twice in a row, our graph becomes false)
    uint16_t command;
    uint16_t compteur=0;        //this counter is used to plot the calibration data
    uint16_t imu=false;         //variable indicating if we are using the imu
    int16_t pos_l_av=0;         //used to memorize the previous value of the left motor
    int16_t pos_r_av=0;         //used to memorize the previous value of the right motor
    int16_t coord_x_av = 0;     //used to memorize the previous value of the x coordinate
    int16_t coord_y_av = 0;     //used to memorize the previous value of the y coordinate
    static int direction=X;     //used to memorize the present direction of the robot
    int distance=0;             //used to memorize the distance the robot has moved after a command
    static uint16_t data[12];   //array we send to the computer
	while(1)
    {
        wait_send_to_epuck();       //wait for a signal from send_receive that the robot received an instruction
            switch(get_state())     //treat the state desired from the computer
            {
                case IDLE:          //neutral case
                    stop_command = true;
                    calibration_done = false;
                    break;
                case CONTROLANDREAD:
                	//send_accelerometre_mesure();    //free the semaphore accelerometre_mesure to check if there has been an impact
                    stop_command = false;
                    calibration_done = false;
                    break;
                case CALIBRATION:
                    if (!calibration_done){
                    while (right_motor_get_pos()>(pos_r_av-(5*mm_to_step(DISTANCE_ONE)))) //calibrate on 50 mm
                        {
                        left_motor_set_speed(-mm_to_step(SPEED)/2); //we go slower to have a better precision
                        right_motor_set_speed(-mm_to_step(SPEED)/2);
                        calibrate();    //get the intensity of the two front proximity sensors
                        data[0] = compteur; //x value to plot on the graph corresponding to the distance the robot has moved
                        data[1] = (get_capteur_values_to_send()[0]+get_capteur_values_to_send()[1])/2;   //y value to plot corresponding to the average intensity of the 2 front proximity sensors
                        SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 2);   //send the 2 values to the computer
                        compteur++;
                        }
                    left_motor_set_speed(0);    //stop the motors when calibration is done
                    right_motor_set_speed(0);
                    distance=0;     //make sure the robot doesn't get wrong coordinates
                    compteur=0;     //reset variable
                    stop_command=true;  //we did not get any command
                    calibration_done = true;
                    }
                    break;
                case LIVEIMU:
                	//send_accelerometre_mesure();     //free the semaphore accelerometre_mesure to plot the acceleration with time
                    calibration_done = false;
                    imu = true;     //we are going to use the imu
                    stop_command = false;   //when we plot the live IMU, the commands are still active
                    break;
            }
        if (!stop_command)  //if the commands aren't blocked
        {
            if (get_impact())    //if an impact is detected
            {
                if (((get_capteur_values_to_send()[0]+get_capteur_values_to_send()[1])/2)>1000)
                    command = TURNAROUND;   //change the command received by the computed to the emergency protocol
                else{
                	command = get_command();
                	reset_impact();
                }
            }
            else
                command = get_command();    //get the command from the value sent by the computer
            switch(command)         //treat the command sent by the computer
            {
                case FORWARD:   //move 10mm forward
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(DISTANCE_ONE)))   //for the same wheel, we wait until it has moved 10mm
                    {
                        left_motor_set_speed(mm_to_step(SPEED));    //the left wheel will move the same distance as the right one
                        right_motor_set_speed(mm_to_step(SPEED));
                    }
                    //left_motor_set_speed(0); Plus de précision?
                    //right_motor_set_speed(0);                    
                    distance = right_motor_get_pos()-pos_r_av;      //calculate the new distance reached by the robot
                    direction=set_direction(FORWARD,direction);
                    send = true;        //the robot has treated the command successfully, we can send the data to the computer
                    break;
                case BACKWARD:  //move 10mm backward
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    while (right_motor_get_pos()>(pos_r_av-mm_to_step(DISTANCE_ONE)))   //for the same wheel, we wait until it has moved -10mm
                    {
                        left_motor_set_speed(-mm_to_step(SPEED));
                        right_motor_set_speed(-mm_to_step(SPEED));
                    }
                    distance = right_motor_get_pos()-pos_r_av;      //calculate the new distance reached by the robot
                    direction=set_direction(BACKWARD,direction);
                    send = true;   //the robot has treated the command successfully, we can send the data to the computer
                    break;
                case LEFT:  //turn 90 degrees to the left
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(NB_COUNTER_QUARTER)))     //for the same wheel, we wait until it has turned 90 degrees
                    {
                    left_motor_set_speed(-mm_to_step(SPEED));   //if the right wheel is moving forward, the left one must move backward in order to turn
                    right_motor_set_speed(mm_to_step(SPEED));
                    }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;     //the robot has turned on himself, it didn't change coordinates
                    direction=set_direction(LEFT,direction);
                    send = true;   //the robot has treated the command successfully, we can send the data to the computer
                    break;
                case RIGHT:     //turn 90 degrees to the right
                    pos_l_av=left_motor_get_pos();      //memorize the position of one wheel to calculate the distance
                    while (left_motor_get_pos()<(pos_l_av+mm_to_step(NB_COUNTER_QUARTER)))  //for the same wheel, we wait until it has turned 90 degrees
                    {
                        left_motor_set_speed(mm_to_step(SPEED));
                        right_motor_set_speed(-mm_to_step(SPEED));      //if the left wheel is moving forward, the right one must move backward in order to turn
                    }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;     //the robot has turned on himself, it didn't change coordinates
                    direction=set_direction(RIGHT,direction);
                    send = true;   //the robot has treated the command successfully, we can send the data to the computer
                    break;
                case NEUTRE:    //the robot stops
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;
                    send = true;   //the robot has treated the command successfully, we can send the data to the computer
                    break;
                case TURNAROUND:    //in case of impact, turn 180 degrees
                    pos_r_av=right_motor_get_pos();     //memorize the position of one wheel to calculate the distance
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(NB_COUNTER_HALF)))    //for the same wheel, we wait until it has turned 180 degrees
                    {
                        left_motor_set_speed(-mm_to_step(SPEED));   //if the right wheel is moving forward, the left one must move backward in order to turn
                        right_motor_set_speed(mm_to_step(SPEED));
                    }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;     //the robot has turned on himself, it didn't change coordinates
                    direction=set_direction(TURNAROUND,direction);
                    send = true;   //the robot has treated the command successfully, we can send the data to the computer
                    reset_impact();
                    break;
            }
            if (send)
            {
                wait_capteur_received();    //wait for the Capteur thread to finish measuring the intensities
                distance = step_to_mm(distance);    //convert distance in mm
                coord_x_av += set_x(distance,direction);    //calculate the new x coordinate
                coord_y_av += set_y(distance,direction);    //calculate the new Y coordinate
                data[0] = coord_x_av;                       //put the values we want to send to the computer in the data array
                data[1] = coord_y_av;                       //
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
                    data[11]= get_acceleration_y()*10;                                 //send the data including the acceleration in y direction
                    SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 12);  //
                    imu = false;    //reset variable
                }
                else
                    SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 11);  //send the data to the computer
                send = false;   //reset variable
            }
        }
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

int16_t mm_to_step(int16_t value_mm){
	double value;
    int16_t value_step;
    value =(double) value_mm * NSTEP_ONE_TURN / WHEEL_PERIMETER;    //equation to convert mm value to step value in double format for precision
    value_step=floor(value + 0.5);  //the values we use are int or uint, we need to round the double value to the upper because the wheels may glide
    return value_step;
}
int16_t step_to_mm(int16_t value_step){
    double value;
    int16_t value_mm;
    value = (double) value_step * WHEEL_PERIMETER / NSTEP_ONE_TURN;    //equation to convert value value to mm value in double format for precision
    value_mm=floor(value + 0.5);  //the values we use are int or uint, we need to round the double value to the upper because the wheels may glide
    return value_mm;
}
