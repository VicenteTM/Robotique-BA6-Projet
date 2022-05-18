#ifndef ACCELEROMETRE
#define ACCELEROMETRE

#include <sensors/imu.h>

#define ACCELERATION_THRESHOLD		5    //for the acceleration value, *10 in order to have a static int instead of a float (unit conversion)

void start_accelerometre(void);
int16_t get_acceleration_y(void);
void get_gravity(imu_msg_t *imu_values);
void wait_accelerometre_mesure(void);
uint8_t get_impact(void);
void reset_impact(void);

#endif /* ACCELEROMETRE */
