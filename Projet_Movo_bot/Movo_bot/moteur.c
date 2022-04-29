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

enum {X,Y,M_X,M_Y};

//values needed for the conversions counter/distance and for the turns
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define NB_COUNTER_HALF  660 // number of step for 180� turn of the motor
#define NB_COUNTER_QUARTER  330 // number of step for 90� turn of the motor theoretically 323 but +7 because the wheels are sliding a little
#define NB_COUNTER_EIGHT  165 // number of step for '(� turn of the motor
//values of the 4 commands
#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3
#define NEUTRE 5

static THD_WORKING_AREA(waMoteur, 1024);
static THD_FUNCTION(Moteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t capteur;
    uint32_t capteur_mm;
    int16_t pos_l_av=0;
    int16_t pos_r_av=0;
    int16_t speed_correction_r = 0;
    int16_t speed_correction_l = 0;
    int direct=X;
	while(1){
		capteur=get_calibrated_prox(FRONT_R_IR);
        speed_correction_r = (get_calibrated_prox(RIGHT_IR)-GOAL_DISTANCE);
        speed_correction_l = (get_calibrated_prox(LEFT_IR)-GOAL_DISTANCE);
        /*
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
            switch(get_command()){
        	case FORWARD:
        		pos_r_av=right_motor_get_pos();
        		while (right_motor_get_pos()<(pos_r_av+77))
        		            {
                left_motor_set_speed(200); // + ROTATION_COEFF*speed_correction_l applies the speed from the command and the correction for the rotation
        	    right_motor_set_speed(200); // + ROTATION_COEFF*speed_correction_r applies the speed from the command and the correction for the rotation
        		            }
        	    chprintf((BaseSequentialStream *)&SD3," forward \r\n"); // send to the computer that we moved 1cm
                break;
            case BACKWARD:
            	pos_r_av=right_motor_get_pos();
            	while (right_motor_get_pos()>(pos_r_av-77))
            	         {
            	     left_motor_set_speed(-300); // + ROTATION_COEFF*speed_correction_l applies the speed from the command and the correction for the rotation
            	     right_motor_set_speed(-300); // + ROTATION_COEFF*speed_correction_r applies the speed from the command and the correction for the rotation
            	  }
            	direct=set_direction(BACKWARD);
        	    chprintf((BaseSequentialStream *)&SD3," backward \r\n");
                break;
            case LEFT:
            	pos_r_av=right_motor_get_pos();
            	while (right_motor_get_pos()<(pos_r_av+NB_COUNTER_QUARTER))
            {
            	   left_motor_set_speed(-200);
            	   right_motor_set_speed(200);
            }
                left_motor_set_speed(0);
        	    right_motor_set_speed(0);
        	    direct=set_direction(LEFT);
                break;
            case RIGHT:
            	pos_l_av=left_motor_get_pos();
            	while (left_motor_get_pos()<(pos_l_av+NB_COUNTER_QUARTER))
            {
            		left_motor_set_speed(200);
            		right_motor_set_speed(-200);
            }
                left_motor_set_speed(0);
        	    right_motor_set_speed(0);
        	    direct=set_direction(RIGHT);
                break;
            case NEUTRE:
                left_motor_set_speed(0);
        	    right_motor_set_speed(0);
        	    chprintf((BaseSequentialStream *)&SD3," stop \r\n");
                break;
        	
       // }
	/*else {
		
		speed_cm_l= left_motor_get_pos() * WHEEL_PERIMETER/ NSTEP_ONE_TURN;
		speed_cm_r= right_motor_get_pos() * WHEEL_PERIMETER/ NSTEP_ONE_TURN;
		chprintf((BaseSequentialStream *)&SD3," left \r\n");
		chprintf((BaseSequentialStream *)&SD3," %d \r\n",left_motor_get_pos());
		chprintf((BaseSequentialStream *)&SD3," %d \r\n",pos_cm_l);
		chprintf((BaseSequentialStream *)&SD3," right \r\n");
		chprintf((BaseSequentialStream *)&SD3," %d \r\n",right_motor_get_pos());
		chprintf((BaseSequentialStream *)&SD3," %d \r\n",pos_cm_r);
	}
*/
        }
	}
}
int set_direction(int move){
    static int direction=X;
    switch (move){
        case LEFT:
        direction--;
        if (direction<X)
        	direction = M_Y;
        break;
        case RIGHT:
        direction++;
        if (direction>M_Y)
           direction = X;
        break;
        case BACKWARD:
        direction+=2;
        if (direction>M_Y)
           direction -= M_Y+1;
    }
    return direction;
}

void start_moteur(void)
{
    chThdCreateStatic(waMoteur, sizeof(waMoteur), NORMALPRIO+2, Moteur, NULL);
}
