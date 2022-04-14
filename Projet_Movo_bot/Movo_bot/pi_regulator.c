#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <mainp.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>
#include <sensors\proximity.h>

enum{FRONT_R_IR,FRONT_RIGHT_IR,RIGHT_IR,BACK_RIGHT_IR,BACK_LEFT_IR,LEFT_IR,FRONT_LEFT_IR,FRONT_L_IR};

#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define DIST_MOTEUR			5 //[cm]
#define MAX_ANGLE			M_PI/2

//simple PI regulator implementation
int16_t pi_regulator(float distance, float goal){

	float error = 0;
	float speed = 0;

	static float sum_error = 0;

	error = distance - goal;

	//disables the PI regulator if the error is to small
	//this avoids to always move as we cannot exactly be where we want and 
	//the camera is a bit noisy
	if(fabs(error) < ERROR_THRESHOLD){
		return 0;
	}

	sum_error += error;

	//we set a maximum and a minimum for the sum to avoid an uncontrolled growth
	if(sum_error > MAX_SUM_ERROR){
		sum_error = MAX_SUM_ERROR;
	}else if(sum_error < -MAX_SUM_ERROR){
		sum_error = -MAX_SUM_ERROR;
	}

	speed = KP * error + KI * sum_error;

    return (int16_t)speed;
}

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

    systime_t time;

    int16_t speed_r = 100*MAX_SUM_ERROR ;
    int16_t speed_correction_r = 0;
    int16_t speed_l = 100*MAX_SUM_ERROR ;
    int16_t speed_correction_l = 0;
    int32_t pos_r = 0;
    int32_t pos_l = 0;
    int32_t pos_avant_r = 0;
    int32_t pos_avant_l = 0;
    int16_t angle=0;
    bool rotate=1;
    //enregistrer première position robot
    while(1){
        time = chVTGetSystemTime();

        
        //computes the speed to give to the motors
        //distance_cm is modified by the image processing thread
       speed_r = -pi_regulator(get_calibrated_prox(FRONT_RIGHT_IR), GOAL_DISTANCE);
       speed_l = -pi_regulator(get_calibrated_prox(FRONT_LEFT_IR), GOAL_DISTANCE);
       //on convertit la position du moteur en coordonnées pour l'afficher sur l'ordi
      /* pos_l=left_motor_get_pos();
       pos_r=right_motor_get_pos();
       pos_l= (pos_l*2*WHEEL_PERIMETER*M_PI)/NSTEP_ONE_TURN;
       pos_r= (pos_r*2*WHEEL_PERIMETER*M_PI)/NSTEP_ONE_TURN;
       pos_avant_l=pos_l-pos_avant_l;
       pos_avant_r=pos_r-pos_avant_r;
       if (abs(speed_r-speed_l)<ERROR_THRESHOLD){
    	   //tout droit
    	   //envoyer la ligne de distance pos_avant_l et r à l'ordi pour le tracer
       }
       else{
    	   if (abs(speed_r)>abs(speed_l))	   	   //virage
    		   angle-=asin(pos_avant_r/(2*DIST_MOTEUR));	   	  //angle en rad
    	   else
    		   angle+=asin(pos_avant_l/(2*DIST_MOTEUR));	   	  //angle en rad
    	   if (abs(angle)>MAX_ANGLE){
    	   if (angle>0){
    	   	   angle-=MAX_ANGLE;
    	   	   speed_correction_l = 0;
    	   }
    	   else{
    	   	   angle+=MAX_ANGLE;
    	   	   speed_correction_r = 0;
    	   }
    	   rotate=0;
    	   }
    	   else
    	   	   rotate=1;

    	   //envoyer l'angle de rotation et la ligne de distance pos_avant_l ou r à l'ordi pour tracer l'arc de cercle
       }
       pos_avant_l=pos_l;
       pos_avant_r=pos_r;
        //computes a correction factor to let the robot rotate to be in front of the line
       if (rotate){
    	   speed_correction_r = (get_calibrated_prox(RIGHT_IR)-GOAL_DISTANCE);
    	   speed_correction_l = (get_calibrated_prox(LEFT_IR)-GOAL_DISTANCE);
       }
       */
       speed_correction_r = (get_calibrated_prox(RIGHT_IR)-GOAL_DISTANCE);
       speed_correction_l = (get_calibrated_prox(LEFT_IR)-GOAL_DISTANCE);

        //if the line is nearly in front of the camera, don't rotate

        //applies the speed from the PI regulator and the correction for the rotation
		right_motor_set_speed(-speed_l + ROTATION_COEFF*speed_correction_r );
		left_motor_set_speed((speed_r + ROTATION_COEFF*speed_correction_l ));

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
