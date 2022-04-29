#ifndef SEND_RECEIVE
#define SEND_RECEIVE

void start_command_send_receive(void);
uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, uint16_t* data, uint16_t size);
void SendUint16ToComputer(BaseSequentialStream* out, uint16_t* data, uint16_t size);
void wait_send_to_epuck(void);
uint16_t get_command(void);

#endif /* SEND_RECEIVE */
