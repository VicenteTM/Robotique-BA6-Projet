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
#include <capteur.h>
 
messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static int impact;

static void timer11_start(void)
{
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
    impact=0;
     while(1)
    {
    	wait_impact();
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));
    	get_gravity(&imu_values);
    }
}

void get_gravity(imu_msg_t *imu_values)
{
    impact = 0;
    //create a pointer to the array for shorter name
    float *accel = imu_values->acceleration;
    impact = accel[Y_AXIS];
}

int get_impact(void)
{
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
    chThdCreateStatic(waImpact, sizeof(waImpact), NORMALPRIO, Impact, NULL);
    //wait 2 sec to be sure the e-puck is in a stable position
    chThdSleepMilliseconds(2000);
}
