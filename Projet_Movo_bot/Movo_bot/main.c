#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
//#include <chprintf.h>
#include <motors.h>
#include <sensors/proximity.h>
//#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <sensors/imu.h>

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
    i2c_start();
    imu_start();
    //starts the serial communication
    serial_start();
    //starts the USB communication
    usb_start();
    //inits the motors
    motors_init();
    start_accelerometre();
    proximity_start();
    calibrate_ir();
    start_moteur();
    start_command_send_receive();
    start_capteur();
    /* Infinite loop. */
    while (1) 
    {
    	//waits 1 second
        chThdSleepMilliseconds(1000);
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
