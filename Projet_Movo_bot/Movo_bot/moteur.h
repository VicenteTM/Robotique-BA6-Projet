#ifndef MOTEUR
#define MOTEUR

void start_moteur(void);
int set_direction(int move, int direction);

uint16_t set_x(uint16_t distance,uint16_t direction);
uint16_t set_y(uint16_t distance,uint16_t direction);
int16_t mm_to_step(int16_t value_mm);
int16_t step_to_mm(int16_t value_step);
void send_data(int16_t coord_x,int16_t coord_y, int direction, uint8_t imu);

//various values needed for the motor thread
#define NSTEP_ONE_TURN      1000 // number of step for 1 turn of the motor
#define WHEEL_PERIMETER     130 // [mm]
#define NB_COUNTER_HALF  84 // number of mm for 180 degrees turn of the motor theoretically (pi*D/2) [mm]
#define NB_COUNTER_QUARTER  42 // number of mm for 90 degrees turn of the motor theoretically (pi*D/4) [mm]
#define SPEED       40     // [mm/s]
#define DISTANCE_ONE    10 // distance to move 1cm [mm]
#define SENSOR_THRESHOLD	1000	// threshold for the sensors
#define ERROR_THRESHOLD		800		// threshold for the adjustment error


//values of the 8 commands
#define NEUTRE 0
#define FORWARD 1
#define BACKWARD 2
#define LEFT 3
#define RIGHT 4
#define ADJUST_LEFT 6
#define ADJUST_RIGHT 7
#define TURNAROUND 5


//values of the 4 states
#define IDLE 0
#define CONTROLANDREAD 1
#define CALIBRATION 2
#define LIVEIMU 3


#endif /* MOTEUR */
