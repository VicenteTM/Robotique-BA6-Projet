#include <ch.h>
#include <hal.h>
#include <math.h>
#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <i2c_bus.h>
#include <sensors/imu.h>
#include <motors.h>
#include <impact.h>
#include <send_receive.h>

#define NB_SAMPLES_OFFSET     200
    
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static int impact;

static void timer11_start(void){
    //General Purpose Timer configuration   
    //timer 11 is a 16 bit timer so we can measure time
    //to about 65ms with a 1Mhz counter
    static const GPTConfig gpt11cfg = {
        1000000,        /* 1MHz timer clock in order to measure uS.*/
        NULL,           /* Timer callback.*/
        0,
        0
    };

    gptStart(&GPTD11, &gpt11cfg);
    //let the timer count to max value
    gptStartContinuous(&GPTD11, 0xFFFF);
}

static THD_WORKING_AREA(waImpact, 1024);
static THD_FUNCTION(Impact, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu");
    imu_msg_t imu_values;
     while(1){
    	 wait_send_to_epuck();
    	 messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	 show_gravity(&imu_values);
    	 //chThdSleepMilliseconds(500);
    }
}

void show_gravity(imu_msg_t *imu_values){

    //we create variables for the led in order to turn them off at each loop and to 
    //select which one to turn on
    uint8_t led1 = 0, led3 = 0, led5 = 0, led7 = 0;
    //threshold value to not use the leds when the robot is too horizontal
    float threshold = 0.8;
    impact = 0;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    //variable to measure the time some functions take
    //volatile to not be optimized out by the compiler if not used
    volatile uint16_t time = 0;

     chSysLock();
     GPTD11.tim->CNT = 0;

     //we find which led of each axis should be turned on
     if(accel[X_AXIS] > threshold){
    	 led7 = 1;
    	 impact = 1;
     }
     else if(accel[X_AXIS] < -threshold){
    	 led3 = 1;
    	 impact = 1;
     }

     if(accel[Y_AXIS] > threshold){
    	 led5 = 1;
    	 impact = 1;
     }

     else if(accel[Y_AXIS] < -threshold){
    	 led1 = 1;
         impact = 1;
     }
    	 //chThdSetPriority(NORMALPRIO+1);

     //if two leds are turned on, turn off the one with the smaller
     //accelerometer value
     if(led1 && led3){
         if(accel[Y_AXIS] < accel[X_AXIS])
             led3 = 0;
         else
             led1 = 0;
     }else if(led3 && led5){
         if(accel[X_AXIS] < -accel[Y_AXIS])
            led5 = 0;
         else
            led3 = 0;
     }else if(led5 && led7){
         if(accel[Y_AXIS] > accel[X_AXIS])
             led7 = 0;
         else
             led5 = 0;
     }else if(led7 && led1){
         if(accel[X_AXIS] > -accel[Y_AXIS])
             led1 = 0;
         else
             led7 = 0;
     }
     time = GPTD11.tim->CNT;
     chSysUnlock();
    //to see the duration on the console
    chprintf((BaseSequentialStream *)&SD3, "time = %dus\n",time);
    //we invert the values because a led is turned on if the signal is low
    palWritePad(GPIOD, GPIOD_LED1, led1 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED3, led3 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED5, led5 ? 0 : 1);
    palWritePad(GPIOD, GPIOD_LED7, led7 ? 0 : 1);

}

int get_impact(void){
	return impact;
}

void impact_start(void)
{
    /* System init */
    timer11_start();
    i2c_start();
    imu_start();
    

    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    //to change the priority of the thread invoking the function. The main function in this case
    //chThdSetPriority(NORMALPRIO+2);
    chThdCreateStatic(waImpact, sizeof(waImpact), NORMALPRIO, Impact, NULL);
    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
    //imu_compute_offset(imu_topic, NB_SAMPLES_OFFSET);

}
