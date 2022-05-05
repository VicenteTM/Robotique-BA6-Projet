#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <sensors\proximity.h>
#include <main.h>
#include <capteur.h>
#include <send_receive.h>

static BSEMAPHORE_DECL(capteur_Received_sem, TRUE);

static uint16_t data_to_send[6];

static THD_WORKING_AREA(waCapteur, 1024);
static THD_FUNCTION(Capteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t capteurNNE,capteurNNO,capteurNE,capteurNO,capteurE,capteurO;
    uint16_t counter=0;
    chprintf((BaseSequentialStream *)&SD3," hey \r\n");
    wait_send_to_epuck();
    capteurNNE=get_calibrated_prox(FRONT_R_IR); //front front right
    data_to_send[0] = counter;
    data_to_send[1] = capteurNNE;
    //SendUint16ToComputer((BaseSequentialStream *) &SD3, data_to_send, 2) ;
    counter += 1;

  	while(1){
            wait_send_to_epuck();
            capteurNNE=get_calibrated_prox(FRONT_R_IR); //front front right
  		    capteurNE=get_calibrated_prox(FRONT_RIGHT_IR); //front right
  		    capteurE=get_calibrated_prox(RIGHT_IR); //right
  		    capteurO=get_calibrated_prox(LEFT_IR); //left
  		    capteurNO=get_calibrated_prox(FRONT_LEFT_IR); //front left
  		    capteurNNO=get_calibrated_prox(FRONT_L_IR); //front front left
            //data_to_send[0] = counter;
            data_to_send[0] = capteurNNE;
            data_to_send[1] = capteurNNO;
            data_to_send[2] = capteurNE;
            data_to_send[3] = capteurNO;
            data_to_send[4] = capteurE;
            data_to_send[5] = capteurO;
            /*
            if (capteurNNE>500)
            	chBSemSignal(&impact_sem);
            else
            	chBSemReset(&impact_sem,1);		//reset the semaphore to taken
            	*/
            //SendUint16ToComputer((BaseSequentialStream *) &SD3, data_to_send, 2);
            //chprintf((BaseSequentialStream *)&SD3,"counter: %d \r\n", data_to_send[0]);
            // chprintf((BaseSequentialStream *)&SD3,"capteur: %d \r\n", data_to_send[1]);
            chBSemSignal(&capteur_Received_sem);
            counter += 1;
  		    // capteur3=get_calibrated_prox(BACK_RIGHT_IR); //back right
  		    // capteur4=get_calibrated_prox(BACK_LEFT_IR); //back left
        
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

uint16_t *get_capteur_values_to_send(void){
	return data_to_send;
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




