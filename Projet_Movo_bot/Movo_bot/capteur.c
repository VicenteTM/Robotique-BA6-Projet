#include "ch.h"
#include "hal.h"
//#include <chprintf.h>
//#include <usbcfg.h>
#include <sensors\proximity.h>
#include <capteur.h>
#include <send_receive.h>

static BSEMAPHORE_DECL(capteur_Received_sem, TRUE);

static uint16_t data_to_send[8];

static THD_WORKING_AREA(waCapteur, 1024);
static THD_FUNCTION(Capteur, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

  	while(1){
            wait_send_to_epuck();
            data_to_send[0] = get_calibrated_prox(FRONT_R_IR); //front front right
            data_to_send[1] = get_calibrated_prox(FRONT_L_IR); //front front left
            data_to_send[2] = get_calibrated_prox(FRONT_RIGHT_IR); //front right
            data_to_send[3] = get_calibrated_prox(FRONT_LEFT_IR); //front left
            data_to_send[4] = get_calibrated_prox(RIGHT_IR); //right
            data_to_send[5] = get_calibrated_prox(LEFT_IR); //left
            data_to_send[6] = get_calibrated_prox(BACK_RIGHT_IR); //right
            data_to_send[7] = get_calibrated_prox(BACK_LEFT_IR); //left
            chBSemSignal(&capteur_Received_sem);
            //waits 0.1 second 
            chThdSleepMilliseconds(100);
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




