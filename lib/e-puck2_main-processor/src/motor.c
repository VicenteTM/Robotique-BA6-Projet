#include <stdlib.h>
#include <stdint.h>
#include <gpio.h>
#include <motor.h>
#include <timer.h>

#define TIMER_CLOCK         84000000
#define TIMER_FREQ          100000 // [Hz]
#define MOTOR_SPEED_LIMIT   13 // [cm/s]
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define NSTEP_ONE_EL_TURN   4  //number of steps to do 1 electrical turn
#define NB_OF_PHASES        4  //number of phases of the motors
#define WHEEL_PERIMETER     13 // [cm]

//timers to use for the motors
#define MOTOR_RIGHT_TIMER       TIM6
#define MOTOR_RIGHT_TIMER_EN    RCC_APB1ENR_TIM6EN
#define MOTOR_RIGHT_IRQHandler  TIM6_DAC_IRQHandler
#define MOTOR_RIGHT_IRQ         TIM6_DAC_IRQn

#define MOTOR_LEFT_TIMER        TIM7
#define MOTOR_LEFT_TIMER_EN     RCC_APB1ENR_TIM7EN
#define MOTOR_LEFT_IRQ          TIM7_IRQn
#define MOTOR_LEFT_IRQHandler   TIM7_IRQHandler

/*
*
*   TO COMPLETE
*   Complete the right GPIO port and pin to be able to control the motors
*/
#define MOTOR_RIGHT_A	GPIOE, 13
#define MOTOR_RIGHT_B	GPIOE, 12
#define MOTOR_RIGHT_C	GPIOE, 14
#define MOTOR_RIGHT_D	GPIOE, 15

#define MOTOR_LEFT_A	GPIOE, 9
#define MOTOR_LEFT_B	GPIOE, 8
#define MOTOR_LEFT_C	GPIOE, 11
#define MOTOR_LEFT_D	GPIOE, 10

#define SPEED_CONTROL 0
#define POSITION_CONTROL 1

/*
*
*   TO COMPLETE
*   step_halt is an array contaning 4 elements describing the state when the motors are off.
*   step_table is an array of 4 lines of 4 elements. Each line describes a step.
*/
static const uint8_t step_halt[NB_OF_PHASES] = {0,0,0,0};
static const uint8_t step_table[NSTEP_ONE_EL_TURN][NB_OF_PHASES] = {
    {1,0,1,0},
    {0,1,1,0},
    {0,1,0,1},
    {1,0,0,1},
};

/*
*
*   Hint :
*   You can declare here static variables which can be used to store the steps counter of the motors
*   for example. They will be available only for the code of this file.
*/
static int16_t counter_l = 0;
static int16_t counter_r = 0;
static int16_t max_counter_l = 0;
static int16_t max_counter_r = 0;
static int16_t right_speed = 0; // in [step/s]
static int16_t left_speed = 0; // in [step/s]
static uint8_t state_motor = 0;
static uint8_t position_right_reached = 0;
static uint8_t position_left_reached = 0;

/*
*
*   TO COMPLETE
*
*   Performs the init of the timers and of the gpios used to control the motors
*/
void motor_init(void)
{
	 SystemClock_Config();

	 // Enable GPIOD and GPIOE peripheral clock
	 RCC->AHB1ENR    |= RCC_AHB1ENR_GPIOEEN | RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOCEN;
	 gpio_config_output_pushpull(MOTOR_RIGHT_A);
	 gpio_config_output_pushpull(MOTOR_RIGHT_B);
	 gpio_config_output_pushpull(MOTOR_RIGHT_C);
	 gpio_config_output_pushpull(MOTOR_RIGHT_D);
	 gpio_config_output_pushpull(MOTOR_LEFT_A);
	 gpio_config_output_pushpull(MOTOR_LEFT_B);
	 gpio_config_output_pushpull(MOTOR_LEFT_C);
	 gpio_config_output_pushpull(MOTOR_LEFT_D);
	 timer6_start();
	 timer7_start();
}

/* b
*
*   TO COMPLETE
*
*   Updates the state of the gpios of the right motor given an array of 4 elements
*   describing the state. For example step_table[0] which gives the first step.
*/
static void right_motor_update(const uint8_t *out)
{
	out[0] ? gpio_set(MOTOR_RIGHT_A) : gpio_clear(MOTOR_RIGHT_A);
	out[1] ? gpio_set(MOTOR_RIGHT_B) : gpio_clear(MOTOR_RIGHT_B);
	out[2] ? gpio_set(MOTOR_RIGHT_C) : gpio_clear(MOTOR_RIGHT_C);
	out[3] ? gpio_set(MOTOR_RIGHT_D) : gpio_clear(MOTOR_RIGHT_D);

}

/*
*
*   TO COMPLETE
*
*   Updates the state of the gpios of the left motor given an array of 4 elements
*   describing the state. For example step_table[0] which gives the first step.
*/
static void left_motor_update(const uint8_t *out)
{
	out[0] ? gpio_set(MOTOR_LEFT_A) : gpio_clear(MOTOR_LEFT_A);
	out[1] ? gpio_set(MOTOR_LEFT_B) : gpio_clear(MOTOR_LEFT_B);
	out[2] ? gpio_set(MOTOR_LEFT_C) : gpio_clear(MOTOR_LEFT_C);
	out[3] ? gpio_set(MOTOR_LEFT_D) : gpio_clear(MOTOR_LEFT_D);

}

/*
*
*   TO COMPLETE
*
*   Stops the motors (all the gpio must be clear to 0) and set 0 to the ARR register of the timers to prevent
*   the interrupts of the timers (because it never reaches 0 after an increment)
*/
void motor_stop(void)
{
	//Set to 0 each phase
	left_motor_update(step_halt);
	right_motor_update(step_halt);
	TIM6->ARR = 0;
	TIM7->ARR = 0;
}

/*
*
*   TO COMPLETE
*
*   Sets the position to reach for each motor.
*   The parameters are in cm for the positions and in cm/s for the speeds.
*/
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l)
{
	counter_l = 0;
	counter_r = 0;
	position_right_reached = 0;
	position_left_reached = 0;
	max_counter_r= -NSTEP_ONE_TURN*position_r/WHEEL_PERIMETER;
	max_counter_l= NSTEP_ONE_TURN*position_l/WHEEL_PERIMETER;
	moteur_set_speed(speed_r, speed_l);
	state_motor = POSITION_CONTROL;
}


uint8_t motor_position_reached(void)
{
if(state_motor == POSITION_CONTROL && position_right_reached && position_left_reached){
return POSITION_REACHED;
}else{
return POSITION_NOT_REACHED;
}
}
/*
*   TO COMPLETE
*
*   Sets the speed of the motors.
*   The parameters are in cm/s for the speed.
*   To set the speed, you need to change the ARR value of the timers.
*   Remember : the timers generate an interrupt when they reach the value of ARR.
*   Don't forget to convert properly the units in order to have the correct ARR value
*   depending on the TIMER_FREQ and the speed chosen.
*/
void moteur_set_speed(float speed_r, float speed_l)
{
	int speed_r_step_s,speed_l_step_s;

	// Limit motor speed
	if (speed_r > MOTOR_SPEED_LIMIT) {
	speed_r = MOTOR_SPEED_LIMIT;
	} else if (speed_r < -MOTOR_SPEED_LIMIT) {
	speed_r = -MOTOR_SPEED_LIMIT;
	}
	if (speed_l > MOTOR_SPEED_LIMIT) {
	speed_l = MOTOR_SPEED_LIMIT;
	} else if (speed_l < -MOTOR_SPEED_LIMIT) {
	speed_l = -MOTOR_SPEED_LIMIT;
	}
	speed_r_step_s = -speed_r * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	speed_l_step_s = speed_l * NSTEP_ONE_TURN / WHEEL_PERIMETER;
	right_speed = speed_r_step_s;
	left_speed = speed_l_step_s;

	//flag for speed control
	state_motor = SPEED_CONTROL;
	// Timer reload takes 1 cycle, this is why -1
	MOTOR_RIGHT_TIMER->ARR = (TIMER_FREQ / abs(speed_r_step_s))-1;
	MOTOR_LEFT_TIMER->ARR = (TIMER_FREQ / abs(speed_l_step_s))-1;
}

/*
*
*   TO COMPLETE
*
*   Interrupt of the timer of the right motor.
*   Performs a step of the motor and stops it if it reaches the position given in motor_set_position().
*/

// // Timer 7 Interrupt Service Routine
void MOTOR_RIGHT_IRQHandler(void)
{
	/*
    *
    *   BEWARE !!
    *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
    *
    *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
    *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
    *
    *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
    *
    */

	/* do something ... */
	static uint8_t i = 0;
	// Check if position is reached
	if ((abs(counter_r) > abs(max_counter_r)) && state_motor == POSITION_CONTROL)
	{
	counter_r = 0;
	position_right_reached = 1;
	right_motor_update(step_halt);
	MOTOR_RIGHT_TIMER->ARR = 0;
	}
	else {
	if (right_speed > 0) {
	i = (i + 1) & 3;
	right_motor_update(step_table[i]);
	counter_r++;
	} else if (right_speed < 0) {
	i = (i - 1) & 3;
	right_motor_update(step_table[i]);
	counter_r--;
	} else {
	right_motor_update(step_halt);
	}
	}
	MOTOR_RIGHT_TIMER->SR &= ~TIM_SR_UIF;
	MOTOR_RIGHT_TIMER->SR; // Read back in order to ensure the effective IF clearing
}

/*
*
*   TO COMPLETE
*
*   Interrupt of the timer of the left motor.
*   Performs a step of the motor and stops it if it reaches the position given in motor_set_position().
*/

// // Timer 6 Interrupt Service Routine
void MOTOR_LEFT_IRQHandler(void)
{
	/*
    *
    *   BEWARE !!
    *   Based on STM32F40x and STM32F41x Errata sheet - 2.1.13 Delay after an RCC peripheral clock enabling
    *
    *   As there can be a delay between the instruction of clearing of the IF (Interrupt Flag) of corresponding register (named here CR) and
    *   the effective peripheral IF clearing bit there is a risk to enter again in the interrupt if the clearing is done at the end of ISR.
    *
    *   As tested, only the workaround 3 is working well, then read back of CR must be done before leaving the ISR
    *
    */

	/* do something ... */
	static uint8_t i = 0;
	// Check if position is reached
	if ((abs(counter_l) > abs(max_counter_l)) && state_motor == POSITION_CONTROL) {
	counter_l = 0;
	position_left_reached = 1;
	left_motor_update(step_halt);
	MOTOR_LEFT_TIMER->ARR = 0;
	}
	else {
	if (left_speed > 0) {
	i = (i + 1) & 3;
	left_motor_update(step_table[i]);
	counter_l++;
	}
	else if (left_speed < 0) {
	i = (i - 1) & 3;
	left_motor_update(step_table[i]);
	counter_l--;
	}
	else {
	left_motor_update(step_halt);
	}
	}
	// Clear interrupt flag
    MOTOR_LEFT_TIMER->SR &= ~TIM_SR_UIF;
    MOTOR_LEFT_TIMER->SR;	// Read back in order to ensure the effective IF clearing
}


