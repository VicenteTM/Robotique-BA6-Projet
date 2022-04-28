#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>

//static global
static uint16_t command[1];

//semaphore
static BSEMAPHORE_DECL(sendToEpuck_sem, TRUE);

// static THD_WORKING_AREA(waReceiveCommand, 1024);
// static THD_FUNCTION(ProcessImage, arg) {

//     chRegSetThreadName(__FUNCTION__);
//     (void)arg;

	
// }

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, uint16_t* data, uint16_t size){

	volatile uint8_t c1, c2;
	volatile uint16_t temp_size = 0;
	//uint16_t i=0;

	uint8_t state = 0;
	while(state != 5){

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state){
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
	c2 = chSequentialStreamGet(in);
	temp_size = (int16_t)(c1);


	if((temp_size/2) == size)
	{
		c1 = chSequentialStreamGet(in);
		c2 = chSequentialStreamGet(in);
		data[0] = (int16_t)(c1);
	}

	return temp_size/2;

}

void SendUint8ToComputer(BaseSequentialStream* out, uint16_t* data, uint16_t size) 
{
	//#ifdef SEND_FROM_MIC
    //waits until a result must be sent to the computer
    //wait_send_to_computer();
	chSequentialStreamWrite(out, (uint8_t*)"START", 5);
	chSequentialStreamWrite(out, (uint8_t*)&size, sizeof(uint16_t));
	chSequentialStreamWrite(out, (uint8_t*)data, sizeof(uint16_t) * size);
}

static THD_WORKING_AREA(waSendReceiveCommand, 1024);
static THD_FUNCTION(SendReceiveCommand, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    

	while(1){
    	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, command, 1);
        //chprintf((BaseSequentialStream *)&SDU1,"Size: %d \r\n",size);
    	if(size == 1)
    	{
			SendUint8ToComputer((BaseSequentialStream *) &SD3, command, 1);
    	//	chprintf((BaseSequentialStream *)&SDU1,"Command: %d \r\n", command[0]);
		chBSemSignal(&sendToEpuck_sem);
    	}
    }
}

void wait_send_to_epuck(void){
	chBSemWait(&sendToEpuck_sem);
}

uint16_t get_command(void){
	return command[0];
}

void start_command_send_receive(void)
{
    chThdCreateStatic(waSendReceiveCommand, sizeof(waSendReceiveCommand), NORMALPRIO, SendReceiveCommand, NULL);
}
