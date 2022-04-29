#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>

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
    uint32_t capteur0;
    uint32_t capteur1;
    uint32_t capteur2;
    uint32_t capteur3;
    uint32_t capteur4;
    uint32_t capteur5;
    uint32_t capteur6;
    uint32_t capteur7;

    chprintf((BaseSequentialStream *)&SD3," hey \r\n");

  	while(1){
  			capteur0=get_calibrated_prox(0); //front front right
  		    capteur1=get_calibrated_prox(1); //front right
  		    capteur2=get_calibrated_prox(2); //right
  		    capteur3=get_calibrated_prox(3); //back right
  		    capteur4=get_calibrated_prox(4); //back left
  		    capteur5=get_calibrated_prox(5); //left
  		    capteur6=get_calibrated_prox(6); //front left
  		    capteur7=get_calibrated_prox(7); //front front left
        chprintf((BaseSequentialStream *)&SD3," test \r\n");
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur0);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur1);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur2);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur3);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur4);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur5);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur6);
        chprintf((BaseSequentialStream *)&SD3," %d \r\n",capteur7);
	}
}


void start_capteur(void)
{
    chThdCreateStatic(waCapteur, sizeof(waCapteur), NORMALPRIO+1, Capteur, NULL);    
}
