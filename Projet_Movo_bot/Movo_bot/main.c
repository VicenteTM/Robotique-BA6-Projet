#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <leds.h>
//#include <arm_math.h>
//#include <chprintf.h>
#include <motors.h>
#include <sensors/proximity.h>
//#include <msgbus/messagebus.h>

#include <send_receive.h>
#include <moteur.h>
#include <capteur.h>
#include <accelerometre.h>

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

int main(void)
{
    halInit();
    chSysInit();
    mpu_init();
    serial_start();   //starts the serial communication
    usb_start();     //starts the USB communication
    start_accelerometre();  //inits the Accelerometre thread
    set_led(LED1,1);
	set_led(LED3,1);
	set_led(LED5,1);
	set_led(LED7,1);
    proximity_start();  //inits the proximity sensors
    calibrate_ir();     //calibrate the proximity sensors
    set_front_led(1);
    start_command_send_receive();   //inits the SendReceive thread
    start_capteur();    //inits the Capteur thread
    motors_init();      //inits the motors
    start_moteur();     //inits the Moteur thread
    /* Infinite loop. */
    while (1) 
    {
        chThdSleepMilliseconds(1000);    	//waits 1 second
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
