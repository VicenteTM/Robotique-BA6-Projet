#include <ch.h>
#include <hal.h>
#include <math.h>
//#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <accelerometre.h>
#include <capteur.h>
#include <moteur.h>

#include <i2c_bus.h>
#include <sensors/imu.h>

messagebus_t bus;           //declaration of the bus used to memorize the imu values
MUTEX_DECL(bus_lock);       //
CONDVAR_DECL(bus_condvar);  //

//static global
static float acceleration_y;     //value of the acceleration in the y direction
static uint8_t impact;

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
    	//wait_accelerometre_mesure();    //attend que la semaphore soit libre, c'est a� dire que le module moteur ait besoin de la valeur
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));  // attend que des nouvelles valeurs de l'imu soient publiees
    	get_gravity(&imu_values);   //recupere la valeur de l'acceleration en y
    	if(acceleration_y>THRESHOLD)
    	    	impact = true;
    }
}

void get_gravity(imu_msg_t *imu_values)
{
    float *accel = imu_values->acceleration;	//create a pointer to the array for shorter name
    acceleration_y = accel[Y_AXIS];

}

float get_acceleration_y(void)
{
	return acceleration_y;  //return the value of the acceleration in the y direction of the robot
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
    i2c_start();    //inits the I2C bu(get_capteur_values_to_send()[0]+get_capteur_values_to_send()[1])/2;s
    imu_start();    //inits the IMU
    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    chThdCreateStatic(waAccelerometre, sizeof(waAccelerometre), NORMALPRIO+1, Accelerometre, NULL); //creation of the Accelerometre thread
    chThdSleepMilliseconds(2000);    //waits two secondes to make sure the epuck is stable
}

void wait_accelerometre_mesure(void)
{
	chBSemWait(&accelerometre_mesure_sem);  //wait until the semaphore is free
}

void send_accelerometre_mesure(void)
{
	chBSemSignal(&accelerometre_mesure_sem);    //free the semaphore accelerometre_mesure to check if there has been an impact
}
