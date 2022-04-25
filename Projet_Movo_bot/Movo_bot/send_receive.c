#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>

#include <main.h>

// static THD_WORKING_AREA(waReceiveCommand, 1024);
// static THD_FUNCTION(ProcessImage, arg) {

//     chRegSetThreadName(__FUNCTION__);
//     (void)arg;

	
// }

uint16_t ReceiveInt16FromComputer(BaseSequentialStream* in, uint16_t* data, uint16_t size){

	volatile uint8_t c1, c2, c3;
	volatile uint16_t temp_size = 0;
	//uint16_t i=0;

	uint8_t state = 0;
	while(state != 5){

        c1 = chSequentialStreamGet(in);

        //State machine to detect the string EOF\0S in order synchronize
        //with the frame received
        switch(state){
        	case 0:
        		if(c1 == 'A')
        			state = 1;
        		else
        			state = 0;
        	case 1:
        		if(c1 == 'B')
        			state = 2;
        		else if(c1 == 'A')
        			state = 1;
        		else
        			state = 0;
        	case 2:
        		if(c1 == 'C')
        			state = 3;
        		else if(c1 == 'A')
        			state = 1;
        		else
        			state = 0;
        	case 3:
        		if(c1 == 'D')
        			state = 4;
        		else if(c1 == 'A')
        			state = 1;
        		else
        			state = 0;
        	case 4:
        		if(c1 == 'E')
        			state = 5;
        		else if(c1 == 'A')
        			state = 1;
        		else
        			state = 0;
        }
        
	}

	c1 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);
	c3 = chSequentialStreamGet(in);
	c2 = chSequentialStreamGet(in);

	int16_t command = (int16_t) (c3);
	temp_size = (int16_t)(c1);


	if((temp_size/2) == size)
	{
		data[0] = command;
	}

	return temp_size/2;

}

static THD_WORKING_AREA(waSendReceiveCommand, 1024);
static THD_FUNCTION(SendReceiveCommand, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    uint16_t command[1];

	while(1){
    	uint16_t size = ReceiveInt16FromComputer((BaseSequentialStream *) &SD3, command, 1);
        chprintf((BaseSequentialStream *)&SDU1,"Size: %d \r\n",size);
    	if(size == 1)
    	{
    		chprintf((BaseSequentialStream *)&SDU1,"Command: %d \r\n", command[0]);
    	}
        chThdSleepMilliseconds(100);
    }
}


void start_command_send_receive(void)
{
    chThdCreateStatic(waSendReceiveCommand, sizeof(waSendReceiveCommand), NORMALPRIO, SendReceiveCommand, NULL);
}
