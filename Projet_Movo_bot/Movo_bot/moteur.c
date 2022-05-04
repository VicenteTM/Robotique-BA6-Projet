#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <motors.h>
#include <moteur.h>
#include <send_receive.h>
#include "capteur.h"
#include <impact.h>

enum {X,Y,M_X,M_Y};

static BSEMAPHORE_DECL(impact_sem, TRUE);

static THD_WORKING_AREA(waMoteur, 1024);
static THD_FUNCTION(Moteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t send=0;
    uint16_t stop=0;
    uint32_t capteur_mm=0;
    uint16_t calibration_done = 0;
    uint compteur=0;
    uint16_t imu=0;
    int16_t pos_l_av=0;
    int16_t pos_r_av=0;
    int16_t coord_x_av = 0;
    int16_t coord_y_av = 0;
    static int direction=X;
    int distance=0;
    static uint16_t data[10];
	while(1){
		/*
		capteur=get_calibrated_prox(FRONT_R_IR);
        speed_correction_r = (get_calibrated_prox(RIGHT_IR)-GOAL_DISTANCE);
        speed_correction_l = (get_calibrated_prox(LEFT_IR)-GOAL_DISTANCE);
        if (capteur>500){
        	chprintf((BaseSequentialStream *)&SD3," wall \r\n");
            pos_l_av=left_motor_get_pos();
            pos_r_av=right_motor_get_pos();
            // chBSemSignal(&sendToComputer_sem);  //push le mouvement ligne droite sur l'ordi avec la distance
            while (left_motor_get_pos()<(pos_l_av+NB_COUNTER_QUARTER)) //Quarter fait un demi tour car le counter est divise par 2 lorsqu'on utilise un signal continu d'ou le fait qu'avec la commande pas besoin
            {
                 left_motor_set_speed(200);
		        right_motor_set_speed(-200);
            }
            chprintf((BaseSequentialStream *)&SD3," wallend \r\n");
            left_motor_set_speed(0);
            right_motor_set_speed(0);
            // chBSemSignal(&sendToComputer_sem);  //push sur l'ordi le virage
        }

        else{
         */
            wait_send_to_epuck();
            /*
            if (get_impact()){
                   	 left_motor_set_speed(0);
                   	 right_motor_set_speed(0);
                   	 chprintf((BaseSequentialStream *)&SD3," impact \r\n");
                }
            else{
            	*/
                switch(get_state()){
                    case IDLE:
                        stop=1;
                        calibration_done=0;
                        break;
                    case CONTROLANDREAD:
                        stop=0;
                        calibration_done=0;
                        palWritePad(GPIOD, GPIOD_LED1,0);
                        palWritePad(GPIOD, GPIOD_LED3,1);
                        palWritePad(GPIOD, GPIOD_LED7,1);
                        palWritePad(GPIOD, GPIOD_LED5,1);
                        break;
                    case CALIBRATION:
                    	if (!calibration_done){
                    		palWritePad(GPIOD, GPIOD_LED1,1);
                    		palWritePad(GPIOD, GPIOD_LED3,0);
                    		palWritePad(GPIOD, GPIOD_LED7,0);
                    		palWritePad(GPIOD, GPIOD_LED5,1);
                        pos_r_av=right_motor_get_pos();
                        while (right_motor_get_pos()>(pos_r_av-385))
        		        	{
                        	left_motor_set_speed(-200); // + ROTATION_COEFF*speed_correction_l applies the speed from the data_received and the correction for the rotation
                        	right_motor_set_speed(-200); // + ROTATION_COEFF*speed_correction_r applies the speed from the data_received and the correction for the rotation
                        	calibrate();
                        	wait_capteur_received();
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
                		palTogglePad(GPIOD, GPIOD_LED1);
                		palTogglePad(GPIOD, GPIOD_LED3);
                		palTogglePad(GPIOD, GPIOD_LED7);
                		palTogglePad(GPIOD, GPIOD_LED5);
                        chBSemSignal(&impact_sem);
                        calibration_done=0;
                        imu=1;
                        stop=0;
                        break;
                }
            if (!stop){
            switch(get_command()){
        	case FORWARD:
        		pos_r_av=right_motor_get_pos();
        		while (right_motor_get_pos()<(pos_r_av+77))
        		            {
                left_motor_set_speed(400); // + ROTATION_COEFF*speed_correction_l applies the speed from the data_received and the correction for the rotation
        	    right_motor_set_speed(400); // + ROTATION_COEFF*speed_correction_r applies the speed from the data_received and the correction for the rotation
        		            }
        		distance = right_motor_get_pos()-pos_r_av;
        		direction=set_direction(FORWARD,direction);
        		send=1;
        	    chprintf((BaseSequentialStream *)&SD3," forward \r\n"); // send to the computer that we moved 1cm
                break;
            case BACKWARD:
            	pos_r_av=right_motor_get_pos();
            	while (right_motor_get_pos()>(pos_r_av-77))
            	         {
            	     left_motor_set_speed(-400); // + ROTATION_COEFF*speed_correction_l applies the speed from the data_received and the correction for the rotation
            	     right_motor_set_speed(-400); // + ROTATION_COEFF*speed_correction_r applies the speed from the data_received and the correction for the rotation
            	  }
            	distance = pos_r_av-right_motor_get_pos();
            	direction=set_direction(BACKWARD,direction);
        	    chprintf((BaseSequentialStream *)&SD3," backward \r\n");
        	    send=1;
                break;
            case LEFT:
            	pos_r_av=right_motor_get_pos();
            	while (right_motor_get_pos()<(pos_r_av+NB_COUNTER_QUARTER))
            {
            	   left_motor_set_speed(-400);
            	   right_motor_set_speed(400);
            }
                left_motor_set_speed(0);
        	    right_motor_set_speed(0);
        	    distance=0;
        	    direction=set_direction(LEFT,direction);
        	    send=1;
                break;
            case RIGHT:
            	pos_l_av=left_motor_get_pos();
            	while (left_motor_get_pos()<(pos_l_av+NB_COUNTER_QUARTER))
            {
            		left_motor_set_speed(400);
            		right_motor_set_speed(-400);
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
        	    chprintf((BaseSequentialStream *)&SD3," stop \r\n");
        	    distance=0;
        	    send=1;
                break;
        }
            if (send){
                wait_capteur_received();
                /*
                data[0] = get_counter_to_send();
                data[1] = get_capteur_right_to_send();
                data[2] = get_capteur_left_to_send();
                data[3] = direction;
                data[4] = get_command();
                */
                distance = distance * 10 * WHEEL_PERIMETER/ NSTEP_ONE_TURN;
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
                if (imu){
                	data[9]= get_impact();
                	SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 10);
                	imu=0;
                	}
                else
                	SendUint16ToComputer((BaseSequentialStream *) &SD3, data, 9);
                send=0;
                }
            // if (abs(get_impact())>THRESHOLD){
                //     pos_r_av=right_motor_get_pos();
                //     while (right_motor_get_pos()<(pos_r_av+NB_COUNTER_HALF))
                //     {
                //     	left_motor_set_speed(-400);
                //     	right_motor_set_speed(400);
                //     }
                //     left_motor_set_speed(0);
                //     right_motor_set_speed(0);
                //     distance=0;
                //     direction=set_direction(BACKWARD,direction);
                //     stop=1;
                //     send=1;
                //             }    
            }
         //}
	}
}

int set_direction(int move,int direction){
	int16_t direction_f=0;
    switch (move){
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
        	direction_f = (direction+2) % 4;
        	break;
    }
    return direction_f;
}

uint16_t set_x(uint16_t distance,uint16_t direction){
    uint16_t distance_f=0;
    switch(direction){
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

uint16_t set_y(uint16_t distance,uint16_t direction){
    uint16_t distance_f=0;
    switch(direction){
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

void wait_impact(void){
	chBSemWait(&impact_sem);
}
