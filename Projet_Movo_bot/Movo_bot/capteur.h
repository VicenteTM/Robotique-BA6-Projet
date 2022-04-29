#ifndef CAPTEUR
#define CAPTEUR

void get_data_to_send(uint16_t **data,uint16_t **datb, uint16_t **datc,int direction);
void start_capteur(void);
void wait_capteur_received(void);
enum{FRONT_R_IR,FRONT_RIGHT_IR,RIGHT_IR,BACK_RIGHT_IR,BACK_LEFT_IR,LEFT_IR,FRONT_LEFT_IR,FRONT_L_IR};
#endif /* CAPTEUR */
