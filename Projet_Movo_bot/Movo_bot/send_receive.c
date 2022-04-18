#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>

// static THD_WORKING_AREA(waReceiveCommand, 1024);
// static THD_FUNCTION(ProcessImage, arg) {

//     chRegSetThreadName(__FUNCTION__);
//     (void)arg;

	
// }

static THD_WORKING_AREA(waSendCommand, 1024);
static THD_FUNCTION(SendCommand, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	while(1){
        chThdSleepMilliseconds(1000);
    	
        chprintf((BaseSequentialStream *)&SD3," hey \r\n");
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
