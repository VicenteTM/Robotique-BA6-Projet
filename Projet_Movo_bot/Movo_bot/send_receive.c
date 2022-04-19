#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <motors.h>

// static THD_WORKING_AREA(waReceiveCommand, 1024);
// static THD_FUNCTION(ProcessImage, arg) {

//     chRegSetThreadName(__FUNCTION__);
//     (void)arg;

	
// }


//MUTEX_DECL(bus_lock);
//CONDVAR_DECL(bus_condvar);

static THD_WORKING_AREA(waSendCommand, 1024);
static THD_FUNCTION(SendCommand, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint32_t capteur;
    uint32_t capteur_mm;
    //messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
    //proximity_msg_t prox_values;
	while(1){
		capteur=get_calibrated_prox(0);
		//if (capteur/=0)
			capteur_mm=30/((float)(capteur));
		//else
			//capteur_mm=30;
        chThdSleepMilliseconds(1000);
        chprintf((BaseSequentialStream *)&SD3," hey \r\n");
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur);
        if (capteur>800){
        	left_motor_set_speed(160);
        	right_motor_set_speed(-160);
        	chprintf((BaseSequentialStream *)&SD3," turn \r\n");
        }

        else{
        	left_motor_set_speed(500);
        	right_motor_set_speed(500);
        	chprintf((BaseSequentialStream *)&SD3," straight \r\n");
        }

    }
}


void start_command_reception(void)
{
    // chThdCreateStatic(waReceiveCommand, sizeof(waReceiveCommand), NORMALPRIO, ReceiveCommand, NULL);    
}


void start_command_send(void)
{
    chThdCreateStatic(waSendCommand, sizeof(waSendCommand), NORMALPRIO, SendCommand, NULL);
}
