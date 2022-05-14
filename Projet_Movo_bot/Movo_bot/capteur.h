#ifndef CAPTEUR
#define CAPTEUR

uint16_t *get_capteur_values_to_send(void);
void start_capteur(void);
void calibrate(void);
void wait_capteur_received(void);
void wait_impact(void);
enum{FRONT_R_IR,FRONT_RIGHT_IR,RIGHT_IR,BACK_RIGHT_IR,BACK_LEFT_IR,LEFT_IR,FRONT_LEFT_IR,FRONT_L_IR};

#endif /* CAPTEUR */
