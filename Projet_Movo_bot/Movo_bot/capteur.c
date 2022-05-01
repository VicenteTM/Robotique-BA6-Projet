#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <capteur.h>
#include <send_receive.h>

static BSEMAPHORE_DECL(capteur_Received_sem, TRUE);

static uint16_t data_to_send[2];

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
    uint16_t counter=0;
    chprintf((BaseSequentialStream *)&SD3," hey \r\n");

    
    wait_send_to_epuck();
    capteur0=get_calibrated_prox(FRONT_R_IR); //front front right
    data_to_send[0] = counter;
    data_to_send[1] = capteur0;
    //SendUint16ToComputer((BaseSequentialStream *) &SD3, data_to_send, 2) ;
    counter += 1;

  	while(1){
            wait_send_to_epuck();
            capteur0=get_calibrated_prox(FRONT_R_IR); //front front right
            data_to_send[0] = counter;
            data_to_send[1] = capteur0;
            //SendUint16ToComputer((BaseSequentialStream *) &SD3, data_to_send, 2);
            //chprintf((BaseSequentialStream *)&SD3,"counter: %d \r\n", data_to_send[0]);
            // chprintf((BaseSequentialStream *)&SD3,"capteur: %d \r\n", data_to_send[1]);
            chBSemSignal(&capteur_Received_sem);
            counter += 1;
  		    
  		    // capteur1=get_calibrated_prox(FRONT_RIGHT_IR); //front right
  		    // capteur2=get_calibrated_prox(RIGHT_IR); //right
  		    // capteur3=get_calibrated_prox(BACK_RIGHT_IR); //back right
  		    // capteur4=get_calibrated_prox(BACK_LEFT_IR); //back left
  		    // capteur5=get_calibrated_prox(LEFT_IR); //left
  		    // capteur6=get_calibrated_prox(FRONT_LEFT_IR); //front left
  		    // capteur7=get_calibrated_prox(FRONT_L_IR); //front front left
        
	}
}

uint16_t get_counter_to_send(void){
	return data_to_send[0];
}

uint16_t get_capteur_right_to_send(void){
	return data_to_send[1];
}

uint16_t get_capteur_left_to_send(void){
	return data_to_send[2];
}

void calibrate(void){
    data_to_send[1] = get_calibrated_prox(FRONT_R_IR); //front front right
    data_to_send[2] = get_calibrated_prox(FRONT_L_IR); //front front left
    chBSemSignal(&capteur_Received_sem);
}

void start_capteur(void)
{
    chThdCreateStatic(waCapteur, sizeof(waCapteur), NORMALPRIO+1, Capteur, NULL);    
}

void wait_capteur_received(void){
	chBSemWait(&capteur_Received_sem);
}
