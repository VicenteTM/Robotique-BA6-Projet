#ifndef SEND_RECEIVE
#define SEND_RECEIVE

void start_command_send_receive(void);
uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, uint16_t* data, uint16_t size);

#endif /* SEND_RECEIVE */
