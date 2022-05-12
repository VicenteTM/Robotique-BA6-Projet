#include "ch.h"
#include "hal.h"
//#include <chprintf.h>
//#include <usbcfg.h>
#include <sensors\proximity.h>

#include <capteur.h>
#include <send_receive.h>

//static global
static uint16_t data_capteur_to_send[8];   //array containing all the intensities of the proximity sensors

//semaphore
static BSEMAPHORE_DECL(capteur_Received_sem, TRUE); //declaration of the semaphore allowing the thread Capteur to get the proximity sensors intensity

//thread
static THD_WORKING_AREA(waCapteur, 1024);
static THD_FUNCTION(Capteur, arg)      //this thread is used to measure the intensity of the proximity sensors and memorize them in an array
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

  	while(1){
            wait_send_to_epuck();
            data_capteur_to_send[0] = get_calibrated_prox(FRONT_R_IR); //get intensity from the front front right proximity sensor
            data_capteur_to_send[1] = get_calibrated_prox(FRONT_L_IR); //get intensity from the front front left proximity sensor
            data_capteur_to_send[2] = get_calibrated_prox(FRONT_RIGHT_IR); //get intensity from the front right proximity sensor
            data_capteur_to_send[3] = get_calibrated_prox(FRONT_LEFT_IR); //get intensity from the front left proximity sensor
            data_capteur_to_send[4] = get_calibrated_prox(RIGHT_IR); //get intensity from the right proximity sensor
            data_capteur_to_send[5] = get_calibrated_prox(LEFT_IR); //get intensity from the left proximity sensor
            data_capteur_to_send[6] = get_calibrated_prox(BACK_RIGHT_IR); //get intensity from the right proximity sensor
            data_capteur_to_send[7] = get_calibrated_prox(BACK_LEFT_IR); //get intensity from the left proximity sensor
            chBSemSignal(&capteur_Received_sem);    //free the semaphore to indicate the robot has measured the values
            chThdSleepMilliseconds(100);	 //waits 0.1 second
	}
}

uint16_t *get_capteur_values_to_send(void)
{
	return data_capteur_to_send;    //return the pointer to the array containing the intensity of the proximity sensors
}

void calibrate(void){
    data_capteur_to_send[1] = get_calibrated_prox(FRONT_R_IR); //get intensity from the front front right proximity sensor
    data_capteur_to_send[2] = get_calibrated_prox(FRONT_L_IR); //get intensity from the front front left proximity sensor
    //chBSemSignal(&capteur_Received_sem);    //free the semaphore to indicate the robot has measured the values
}

void start_capteur(void)
{
    chThdCreateStatic(waCapteur, sizeof(waCapteur), NORMALPRIO+2, Capteur, NULL); //creation of the Capteur thread
}

void wait_capteur_received(void){
	chBSemWait(&capteur_Received_sem);  //wait until the semaphore is free
}




