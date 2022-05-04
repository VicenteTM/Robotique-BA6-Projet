#ifndef MOTEUR
#define MOTEUR

void start_moteur(void);
int set_direction(int move, int direction);
void wait_impact(void);
uint16_t set_x(uint16_t distance,uint16_t direction);
uint16_t set_y(uint16_t distance,uint16_t direction);
#define ROTATION_COEFF			2.5
#define GOAL_DISTANCE 			300.0f

//values needed for the conversions counter/distance and for the turns
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     13 // [cm]
#define NB_COUNTER_HALF  660 // number of step for 180� turn of the motor
#define NB_COUNTER_QUARTER  330 // number of step for 90� turn of the motor theoretically 323 but +7 because the wheels are sliding a little
#define NB_COUNTER_EIGHT  165 // number of step for '(� turn of the motor
//values of the 4 commands
#define NEUTRE 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4

#define IDLE 0
#define CONTROLANDREAD 1
#define CALIBRATION 2
#define LIVEIMU 3


#endif /* MOTEUR */
