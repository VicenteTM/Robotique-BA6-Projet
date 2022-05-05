#ifndef IMPACT
#define IMPACT

#include <sensors/imu.h>

void impact_start(void);
int get_impact(void);
void get_gravity(imu_msg_t *imu_values);

#endif /* IMPACT */
