#sends the data of the sinus to the serial port in int8
import sys
import struct
from threading import Thread
import time
import numpy as np
import serial
from captorsPlot import CaptorDist, LiveIMU
from robotPlot import Robot

NEUTRAL = 5
FORWARD = 0
BACKWARD = 1
LEFT = 2
RIGHT = 3
DCCALIBRATION = 6

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
        data = [x[0] for x in data]
        return data
    else:
        print('Timout...')
        return []


def readfromrobot(port):
    value = readUInt16Serial(port) 
    if(len(value)>0):
        print(value)
    return value

# #thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port, robot):
        Thread.__init__(self)
        self.robot = robot
        self.contReceiveCaptorD = False
        self.contReceiveIMU = False
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
                newvalues = readfromrobot(self.port)
                self.robot.add_captor_caption(newvalues)
            elif(self.contReceiveCaptorD):
                sendRobotCommand(self.port, self.robot.command)
                newvalues = readfromrobot(self.port) 
                # newvalues = [np.random.randint(0,2000),np.random.randint(0,4000)]
                if not newvalues == []:
                    self.captorD.addValues(newvalues)
                    dist, intensity = self.captorD.get_values()
                    self.line_capt_d.set_xdata(dist)
                    self.line_capt_d.set_ydata(intensity)
            elif(self.contReceiveIMU):
                sendRobotCommand(self.port, self.robot.command)
                newvalue = readfromrobot(self.port)
                if not newvalue == []: 
                    #newvalue = np.random.randint(0,30*100)/100
                    self.liveIMU.addValue(newvalue[0])
                    time_l, intensity = self.liveIMU.get_values()
                    self.line_live_IMU.set_xdata(time_l)
                    self.line_live_IMU.set_ydata(intensity)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    #and disables the continuous sending and receiving
    def setContReceiveCaptorD(self, line_capt_d):  
        self.contSendAndReceive = False
        self.contReceiveCaptorD = True
        self.contReceiveIMU = False
        self.line_capt_d = line_capt_d
        self.captorD = CaptorDist()

    #disables the continuous reading
    #and enables the continuous sending and receiving
    def setContSendAndReceive(self):
        self.contSendAndReceive = True
        self.contReceiveCaptorD = False
        self.contReceiveIMU = False
        
    def setContLiveIMU(self, line_live_IMU):
        self.contSendAndReceive = False
        self.contReceiveCaptorD = False
        self.contReceiveIMU = True
        self.line_live_IMU = line_live_IMU
        self.liveIMU = LiveIMU()

    #disables the continuous reading
    #and disables the continuous sending and receiving
    def stop_reading(self):
        self.contSendAndReceive = False
        self.contReceiveCaptorD = False
        self.contReceiveIMU = False

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