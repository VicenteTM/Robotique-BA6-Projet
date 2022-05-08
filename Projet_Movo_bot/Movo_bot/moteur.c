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

enum {X,Y,M_X,M_Y};     //enum des 4 directions pour pouvoir indiquer à l'ordinateur le sens du robot ainsi que le calcul des coordonnées

static BSEMAPHORE_DECL(accelerometre_sem, TRUE);   //déclaration de la sémaphore permettant de récupérer la valeur de l'accéleration en y

static THD_WORKING_AREA(waMoteur, 1024);    //ce thread permet de gérer les commandes et états et fait tourner les roues en conséquence, il gère aussi l'envoi de données à l'ordinateur
static THD_FUNCTION(Moteur, arg) 
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t send = 1;
    uint16_t stop = 0;
    uint16_t calibration_done = 0;
    uint16_t command;
    uint16_t compteur=0;
    uint16_t imu=0;
    int16_t pos_l_av=0;
    int16_t pos_r_av=0;
    int16_t coord_x_av = 0;
    int16_t coord_y_av = 0;
    static int direction=X;
    int distance=0;
    static uint16_t data[12];
	while(1)
    {
        wait_send_to_epuck();
            switch(get_state())
            {
                case IDLE:
                    stop=1;
                    calibration_done=0;
                    break;
                case CONTROLANDREAD:
                	chBSemSignal(&accelerometre_sem);
                    stop=0;
                    calibration_done=0;
                    break;
                case CALIBRATION:
                    if (!calibration_done){
                    while (right_motor_get_pos()>(pos_r_av-(5*mm_to_step(DISTANCE_ONE)))) //calibrate on 50mm
                        {
                        left_motor_set_speed(-mm_to_step(SPEED)/2); //we go slower to have a better precision
                        right_motor_set_speed(-mm_to_step(SPEED)/2);
                        calibrate();
                        data[0] = compteur;
                        data[1] = (get_capteur_left_to_send()+get_capteur_right_to_send())/2;
                        SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 2);
                        compteur++;
                        }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;
                    send=0;
                    compteur=0;
                    stop=1;
                    calibration_done=1;
                    }
                    break;
                case LIVEIMU:
                    chBSemSignal(&accelerometre_sem);
                    calibration_done=0;
                    imu=1;
                    stop=0;
                    break;
            }
        if (!stop)
        {
            if (abs(get_acceleration_y())>THRESHOLD)
                command = TURNAROUND;
            else
                command = get_command();
            switch(command)
            {
                case FORWARD:
                    pos_r_av=right_motor_get_pos();
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(DISTANCE_ONE)))
                    {
                        left_motor_set_speed(mm_to_step(SPEED));
                        right_motor_set_speed(mm_to_step(SPEED));
                    }
                    distance = right_motor_get_pos()-pos_r_av;
                    direction=set_direction(FORWARD,direction);
                    send=1;
                    break;
                case BACKWARD:
                    pos_r_av=right_motor_get_pos();
                    while (right_motor_get_pos()>(pos_r_av-mm_to_step(DISTANCE_ONE)))
                    {
                        left_motor_set_speed(-mm_to_step(SPEED));
                        right_motor_set_speed(-mm_to_step(SPEED));
                    }
                    distance = right_motor_get_pos()-pos_r_av;
                    direction=set_direction(BACKWARD,direction);
                    send=1;
                    break;
                case LEFT:
                    pos_r_av=right_motor_get_pos();
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(NB_COUNTER_QUARTER)))
                    {
                    left_motor_set_speed(-mm_to_step(SPEED));
                    right_motor_set_speed(mm_to_step(SPEED));
                    }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;
                    direction=set_direction(LEFT,direction);
                    send=1;
                    break;
                case RIGHT:
                    pos_l_av=left_motor_get_pos();
                    while (left_motor_get_pos()<(pos_l_av+mm_to_step(NB_COUNTER_QUARTER)))
                    {
                        left_motor_set_speed(mm_to_step(SPEED));
                        right_motor_set_speed(-mm_to_step(SPEED));
                    }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;
                    direction=set_direction(RIGHT,direction);
                    send=1;
                    break;
                case NEUTRE:
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;
                    send=1;
                    break;
                case TURNAROUND:
                    pos_r_av=right_motor_get_pos();
                    while (right_motor_get_pos()<(pos_r_av+mm_to_step(NB_COUNTER_HALF)))
                    {
                        left_motor_set_speed(-mm_to_step(SPEED));
                        right_motor_set_speed(mm_to_step(SPEED));
                    }
                    left_motor_set_speed(0);
                    right_motor_set_speed(0);
                    distance=0;
                    direction=set_direction(TURNAROUND,direction);
                    send=1;
                    break;
            }
            if (send)
            {
                wait_capteur_received();
                distance = step_to_mm(distance);
                coord_x_av += set_x(distance,direction);
                coord_y_av += set_y(distance,direction);
                data[0] = coord_x_av;
                data[1] = coord_y_av;
                data[2] = direction;
                data[3] = get_capteur_values_to_send()[0];
                data[4] = get_capteur_values_to_send()[1];
                data[5] = get_capteur_values_to_send()[2];
                data[6] = get_capteur_values_to_send()[3];
                data[7] = get_capteur_values_to_send()[4];
                data[8] = get_capteur_values_to_send()[5];
                data[9] = get_capteur_values_to_send()[6];
                data[10] = get_capteur_values_to_send()[7];
                if (imu)
                {
                    data[11]= get_acceleration_y();
                    SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 12);
                    imu=0;
                }
                else
                    SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 11);
                send=0;
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
    		direction_f = direction;
    		break;
        case RIGHT:
        	direction_f = (direction-1) % 4;
        	if (direction_f == -1)
        		direction_f = M_Y;
        	break;
        case LEFT:
        	direction_f = (direction+1) % 4;
        	break;
        case BACKWARD:
        	direction_f = direction;
        	break;
        case TURNAROUND:
        	direction_f = (direction+2) % 4;
        	break;
    }
    return direction_f;
}

uint16_t set_x(uint16_t distance,uint16_t direction)
{
    uint16_t distance_f=0;
    switch(direction)
    {
        case X:
            distance_f=distance;
            break;
        case M_X:
            distance_f= -distance;
            break;
        case Y:
            distance_f=0;
            break;
        case M_Y:
            distance_f= 0;
            break;
    }
    return distance_f;
}

uint16_t set_y(uint16_t distance,uint16_t direction)
{
    uint16_t distance_f=0;
    switch(direction)
    {
        case X:
            distance_f=0;
            break;
        case M_X:
            distance_f= 0;
            break;
        case Y:
            distance_f = distance;
            break;
        case M_Y:
            distance_f= -distance;
            break;
    }
    return distance_f;
}

void start_moteur(void)
{
    chThdCreateStatic(waMoteur, sizeof(waMoteur), NORMALPRIO+2, Moteur, NULL);
}

void wait_accelerometre(void)
{
	chBSemWait(&accelerometre_sem);
}

int16_t mm_to_step(int16_t value_mm){
	double value;
    int16_t value_step;
    value =(double) value_mm * NSTEP_ONE_TURN / WHEEL_PERIMETER;
    value_step=floor(value + 0.5);
    return value_step;
}
int16_t step_to_mm(int16_t value_step){
    double value;
    int16_t value_mm;
    value = (double) value_step * WHEEL_PERIMETER / NSTEP_ONE_TURN;
    value_mm=floor(value + 0.5);
    return value_mm;
}
