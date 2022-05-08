#include <ch.h>
#include <hal.h>
#include <math.h>
//#include <chprintf.h>
#include <msgbus/messagebus.h>
#include <accelerometre.h>
#include <capteur.h>
#include <moteur.h>

messagebus_t bus;           //declaration of the bus used to memorize the imu values
MUTEX_DECL(bus_lock);       //
CONDVAR_DECL(bus_condvar);  //

//static global
static int acceleration_y;     //value of the acceleration in the y direction

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
     while(1)
    {
    	wait_accelerometre_mesure();    //attend que la semaphore soit libre, c'est a� dire que le module moteur ait besoin de la valeur
    	messagebus_topic_wait(imu_topic, &imu_values, sizeof(imu_values));  // attend que des nouvelles valeurs de l'imu soient publiees
    	get_gravity(&imu_values);   //recupere la valeur de l'acceleration en y
    }
}

void get_gravity(imu_msg_t *imu_values)
{
    float *accel = imu_values->acceleration;	//create a pointer to the array for shorter name
    acceleration_y = accel[Y_AXIS];
}

int get_acceleration_y(void)
{
	return acceleration_y;  //return the value of the acceleration in the y direction of the robot
}

void start_accelerometre(void)
{
    /* System init */
    timer11_start();
    /** Inits the Inter Process Communication bus. */
    messagebus_init(&bus, &bus_lock, &bus_condvar);
    chThdCreateStatic(waAccelerometre, sizeof(waAccelerometre), NORMALPRIO, Accelerometre, NULL); //creation of the Accelerometre thread
    chThdSleepMilliseconds(2000);    //waits two secondes to make sure the epuck is stable
}
