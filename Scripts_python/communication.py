#sends the data of the sinus to the serial port in int8
import sys
import struct
from threading import Thread
import time
import numpy as np
import serial
from robotPlot import Robot

NEUTRAL = 5
FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3

def sendRobotCommand(port,data_to_send):
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
def readUInt16Serial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...???')
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
    rcv_buffer = port.read(size*2)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == 2*size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<h',rcv_buffer, i*2))
            i = i+1

        print('received !')
        return data
    else:
        print('Timout...')
        return []


def readcommand(port):
    command = readUInt16Serial(port) 
    if(len(command)>0):
        print(command)

# #thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port, robot):
        Thread.__init__(self)
        self.robot = robot
        self.contReceiveCaptor = False
        self.contSendAndReceive = False
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))
        
        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)
    #function called after the init
    def run(self):
        
        while(self.alive):
            if(self.contSendAndReceive):
                sendRobotCommand(self.port, self.robot.command)
                time.sleep(0.3)

            elif(self.contReceiveCaptorD):
                readcommand(self.port)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    #and disables the continuous sending and receiving
    def setContReceiveCaptorD(self, val):  
        self.contSendAndReceive = False
        self.contReceiveCaptorD = True

    #disables the continuous reading
    #and enables the continuous sending and receiving
    def setContSendAndReceive(self, val):
        self.contSendAndReceive = True
        self.contReceiveCaptorD = False

    #disables the continuous reading
    #and disables the continuous sending and receiving
    def stop_reading(self, val):
        self.contSendAndReceive = False
        self.contReceiveCaptorD = False

    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()