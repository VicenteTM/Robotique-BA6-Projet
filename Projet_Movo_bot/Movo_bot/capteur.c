#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <sensors\proximity.h>

// static THD_WORKING_AREA(waReceiveCommand, 1024);
// static THD_FUNCTION(ProcessImage, arg) {

//     chRegSetThreadName(__FUNCTION__);
//     (void)arg;

	
// }


//MUTEX_DECL(bus_lock);
//CONDVAR_DECL(bus_condvar);

static THD_WORKING_AREA(waCapteur, 1024);
static THD_FUNCTION(Capteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint32_t capteur;
    uint32_t capteur_mm;
    capteur=get_calibrated_prox(0);
    chprintf((BaseSequentialStream *)&SD3," hey \r\n");
    chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur);
  	while(1){
        chThdSleepMilliseconds(1000);
    	
        chprintf((BaseSequentialStream *)&SD3," hey \r\n");

	}
}


void start_capteur(void)
{
    chThdCreateStatic(waCapteur, sizeof(waCapteur), NORMALPRIO+1, Capteur, NULL);    
}