#sends the data of the sinus to the serial port in int8
import struct
import time
import numpy as np


def sendFloatSerial(port,data_to_send):
    data = np.array([data_to_send]).astype(np.int16)

    #to convert to int16 we need to pass via numpy
    size = np.array([data.size], dtype=np.int16)

    send_buffer = bytearray([])

    i = 0
    while(i < size[0]):
        send_buffer += struct.pack('<h',data[i])
        i = i+1
    

    port.write(b'START')
    port.write(struct.pack('<h',2*size[0]))
    port.write(send_buffer)
    print('sent !')

# #reads the FFT in float32 from the serial
def readFloatSerial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...')
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size*4)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == 4*size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<f',rcv_buffer, i*4))
            i = i+1

        print('received !')
        return data
    else:
        print('Timout...')
        return []