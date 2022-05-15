#include <ch.h>
#include <hal.h>
#include <math.h>
#include <msgbus/messagebus.h>
#include <accelerometre.h>

#include <i2c_bus.h>
#include <sensors/imu.h>

messagebus_t bus;           //declaration of the bus used to memorize the imu values
MUTEX_DECL(bus_lock);       //
CONDVAR_DECL(bus_condvar);  //

//static global
static int16_t acceleration_y;    //value of the acceleration in the y direction
static uint8_t impact;          //variable indicating the state of the impact

//semaphore
static BSEMAPHORE_DECL(accelerometre_mesure_sem, TRUE);   //declaration of the semaphore allowing the thread Accelerometre to get the value of the y acceleration


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

//thread
static THD_WORKING_AREA(waAccelerometre, 1024);
static THD_FUNCTION(Accelerometre, arg)        //this thread is used to measure the acceleration of the IMU and memorize them in a static int value
{

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    messagebus_topic_t *imu_topic = messagebus_find_topic_blocking(&bus, "/imu"); 
    imu_msg_t imu_values;
    acceleration_y=0;
    impact=false;
     while(1)
    {
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));  // wait for new values of the imu to be published
    	get_gravity(&imu_values);   //get the value of the y acceleration
    	if(acceleration_y>ACCELERATION_THRESHOLD)    //if there has been an impact, the y value exceeds the threshold
    	    impact = true;
        chBSemSignal(&accelerometre_mesure_sem);    //free the semaphore accelerometre_mesure to indicate the accelerometer value has been refreshed
    }
}

void get_gravity(imu_msg_t *imu_values)
{
    float acceleration_float_to_int = 0;    //transition variable 
    float *accel = imu_values->acceleration;	//create a pointer to the array for shorter name
    acceleration_float_to_int = accel[Y_AXIS];
    acceleration_y = acceleration_float_to_int*10; //*10 in order to have a static int instead of a float (unit conversion)

}

int16_t get_acceleration_y(void)
{
	return acceleration_y;  //return the value of the acceleration in the y direction of the robot as int value because we need it to send to the computer (*10 is to see the variations better)
}

uint8_t get_impact(void)
{
	return impact;  //return if the impact has happened or not
}

void reset_impact(void)
{
	impact=false;  //reset impact to default value
}

void start_accelerometre(void)
{
    /* System init */
    timer11_start();
    i2c_start();    //inits the I2C bus
    imu_start();    //inits the IMU
    messagebus_init(&bus, &bus_lock, &bus_condvar);    //Inits the Inter Process Communication bus
    chThdCreateStatic(waAccelerometre, sizeof(waAccelerometre), NORMALPRIO+1, Accelerometre, NULL); //creation of the Accelerometre thread
    chThdSleepMilliseconds(2000);    //waits two secondes to make sure the epuck is stable
}

void wait_accelerometre_mesure(void)
{
	chBSemWait(&accelerometre_mesure_sem);  //wait until the semaphore is free
}

