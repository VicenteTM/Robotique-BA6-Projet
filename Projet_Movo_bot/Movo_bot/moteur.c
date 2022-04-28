#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <motors.h>


#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define NB_COUNTER_HALF     330 // number of step for 180° turn of the motor theoretically 323 but +7 because the wheels are sliding a little
#define NB_COUNTER_QUARTER  165 // number of step for 90° turn of the motor

static THD_WORKING_AREA(waMoteur, 1024);
static THD_FUNCTION(Moteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint32_t capteur;
    uint32_t capteur_mm;
    int32_t pos_l_av=0;
    int32_t pos_r_av=0;

	while(1){
		capteur=get_calibrated_prox(0);
        chprintf((BaseSequentialStream *)&SD3," hey \r\n");
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur);
        if (capteur>800){
        	chprintf((BaseSequentialStream *)&SD3," turn \r\n");
            pos_l_av=left_motor_get_pos(); //push le mouvement ligne droite sur l'ordi avec la distance
            pos_r_av=right_motor_get_pos();

            while (left_motor_get_pos()<(pos_l_av+NB_COUNTER_HALF))
            {
                 left_motor_set_speed(200);
		        right_motor_set_speed(-200);
            }
            	//push sur l'ordi qu'on a fait un tour
        }
        else{
        	left_motor_set_speed(500);
        	right_motor_set_speed(500);
        	chprintf((BaseSequentialStream *)&SD3," straight \r\n");
        }
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

void start_moteur(void)
{
    chThdCreateStatic(waMoteur, sizeof(waMoteur), NORMALPRIO+1, Moteur, NULL);
}
