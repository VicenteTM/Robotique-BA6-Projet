#ifndef MOTEUR
#define MOTEUR

void start_moteur(void);
int set_direction(int move, int direction);
#define ROTATION_COEFF			2.5
#define GOAL_DISTANCE 			300.0f

#endif /* MOTEUR */
