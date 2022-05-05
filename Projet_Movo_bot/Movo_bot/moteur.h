#ifndef MOTEUR
#define MOTEUR

void start_moteur(void);
int set_direction(int move, int direction);
void wait_impact(void);
uint16_t set_x(uint16_t distance,uint16_t direction);
uint16_t set_y(uint16_t distance,uint16_t direction);
int16_t mm_to_step(int16_t value_mm);
int16_t step_to_mm(int16_t value_step);

#define ROTATION_COEFF			2.5
#define GOAL_DISTANCE 			30.0f
#define THRESHOLD				1.0f

//values needed for the conversions counter/distance and for the turns
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     130 // [mm]
#define NB_COUNTER_HALF  84 // number of step for 180� turn of the motor theoretically (pi*D/2) [mm]
#define NB_COUNTER_QUARTER  42 // number of mm for 90� turn of the motor theoretically (pi*D/4) [mm]
#define SPEED       52      // [mm/s]
#define DISTANCE_ONE    10 // distance pour faire un cm [mm]
//values of the 4 commands
#define NEUTRE 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define TURNAROUND 5

#define IDLE 0
#define CONTROLANDREAD 1
#define CALIBRATION 2
#define LIVEIMU 3


#endif /* MOTEUR */
