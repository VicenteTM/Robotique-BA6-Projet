#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <capteur.h>
#include <send_receive.h>

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
    uint16_t capteur0;
    static int counter=0;
    uint16_t data_to_send[2];
    chprintf((BaseSequentialStream *)&SD3," hey \r\n");

    
    wait_send_to_epuck();
    capteur0=get_calibrated_prox(FRONT_R_IR); //front front right
    data_to_send[0] = counter;
    data_to_send[1] = capteur0;
    SendUint8ToComputer((BaseSequentialStream *) &SD3, data_to_send, 2) ;
    counter += 1;

  	while(1){
            wait_send_to_epuck();
            if (get_command()==1){
                capteur0=get_calibrated_prox(FRONT_R_IR); //front front right
                data_to_send[0] = counter;
                data_to_send[1] = capteur0;
                SendUint8ToComputer((BaseSequentialStream *) &SD3, data_to_send, 2);
                counter += 1;
            }
  		    
  		    // capteur1=get_calibrated_prox(FRONT_RIGHT_IR); //front right
  		    // capteur2=get_calibrated_prox(RIGHT_IR); //right
  		    // capteur3=get_calibrated_prox(BACK_RIGHT_IR); //back right
  		    // capteur4=get_calibrated_prox(BACK_LEFT_IR); //back left
  		    // capteur5=get_calibrated_prox(LEFT_IR); //left
  		    // capteur6=get_calibrated_prox(FRONT_LEFT_IR); //front left
  		    // capteur7=get_calibrated_prox(FRONT_L_IR); //front front left
        
	}
}


void start_capteur(void)
{
    chThdCreateStatic(waCapteur, sizeof(waCapteur), NORMALPRIO+1, Capteur, NULL);    
}
