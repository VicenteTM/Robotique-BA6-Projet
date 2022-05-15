#include "ch.h"
#include "hal.h"
//#include <chprintf.h>
#include <usbcfg.h>
#include <send_receive.h>

//static global
static uint16_t data_received[2];	 //array containing the instructions of the computer

//semaphore
static BSEMAPHORE_DECL(sendToEpuck_sem, TRUE);   //declaration of the semaphore allowing the threads to access the instructions of the computer

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, uint16_t* data, uint16_t size)
{

	volatile uint8_t c1;
	volatile uint16_t temp_size = 0;

	uint8_t state = 0;
	while(state != 5)
	{

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state)
		{
        	case 0:
        		if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 1:
        		if(c1 == 'T')
        			state = 2;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 2:
        		if(c1 == 'A')
        			state = 3;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 3:
        		if(c1 == 'R')
        			state = 4;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        	case 4:
        		if(c1 == 'T')
        			state = 5;
        		else if(c1 == 'S')
        			state = 1;
        		else
        			state = 0;
        		break;
        }

	}

	c1 = chSequentialStreamGet(in);
	chSequentialStreamGet(in);
	temp_size = (int16_t)(c1);


	if((temp_size/2) == size)
	{
		c1 = chSequentialStreamGet(in);
		chSequentialStreamGet(in);
		data[0] = (int16_t)(c1);
		c1 = chSequentialStreamGet(in);
		chSequentialStreamGet(in);
		data[1] = (int16_t)(c1);
	}

	return temp_size/2;

}

void SendUint16ToComputer(BaseSequentialStream* out, uint16_t* data, uint16_t size) 
{
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(uint16_t) * size);
}

static THD_WORKING_AREA(waSendReceiveCommand, 1024);
static THD_FUNCTION(SendReceiveCommand, arg) 
{
    chRegSetThreadName(__FUNCTION__);
    (void)arg;
	while(1)
	{
    	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, data_received, 2);
    	if(size == 2)
    	{
			chBSemSignal(&sendToEpuck_sem);		//free the semaphore to indicate the robot has received the computer's instructions
    	}
    }
}

void wait_send_to_epuck(void)
{
	chBSemWait(&sendToEpuck_sem);  //wait until the semaphore is free
}

uint16_t get_state(void)
{
	return data_received[0];	//return the value of the state sent by the computer
}

uint16_t get_command(void)
{
	return data_received[1];	//return the value of the command sent by the computer
}

void start_command_send_receive(void)
{
    chThdCreateStatic(waSendReceiveCommand, sizeof(waSendReceiveCommand), NORMALPRIO+3, SendReceiveCommand, NULL);	//creation of the SendReceive thread 
}
