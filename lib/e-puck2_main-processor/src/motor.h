#ifndef MOTOR_H
#define MOTOR_H

#define POSITION_NOT_REACHED 0
#define POSITION_REACHED 1

void motor_init(void);
void motor_set_speed(float speed_r, float speed_l);
void motor_set_position(float position_r, float position_l, float speed_r, float speed_l);
void motor_stop(void);
uint8_t motor_position_reached(void);


#endif /* MOTOR_H */
