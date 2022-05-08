#ifndef ACCELEROMETRE
#define ACCELEROMETRE

#include <sensors/imu.h>

void start_accelerometre(void);
int get_acceleration_y(void);
void get_gravity(imu_msg_t *imu_values);

#endif /* ACCELEROMETRE */
