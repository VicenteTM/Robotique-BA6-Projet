#sends the data of the sinus to the serial port in int8
import sys
import struct
from threading import Thread
import time
import numpy as np
import serial
from movobot_lib.captorsPlot import CaptorDist, LiveIMU
from movobot_lib.robotPlot import Robot

NEUTRAL = 0
FORWARD = 1
BACKWARD = 2
LEFT = 3
RIGHT = 4

IDLE = 0
CONTROLANDREAD = 1
DCCALIBRATION = 2
LIVEIMU = 3

def sendRobotCom(port,data_to_send):
    data = np.array(data_to_send).astype(np.int16)

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
    else:
        print("[]")
    return value

# #thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port, robot):
        Thread.__init__(self)
        self.robot = robot
        self.alive = True
        self.need_to_update = False
        self.commun_state = IDLE

        print('Connecting to port {}'.format(port))
        
        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    #function called after the init
    def run(self):
        while(self.alive):
            sendRobotCom(self.port, [self.commun_state,self.robot.command])
            if self.commun_state in [CONTROLANDREAD, DCCALIBRATION, LIVEIMU]:
                self.buffer_clean_opti()
                received_data = readfromrobot(self.port)
                if self.check_length(len(received_data)):
                    if self.commun_state == CONTROLANDREAD:
                        self.robot.update_robot_from_reception(received_data)

                    elif self.commun_state == DCCALIBRATION:
                        self.captorD.addValues(received_data)
                        dist, intensity = self.captorD.get_values()

                        self.line_capt_d.set_xdata(dist)
                        self.line_capt_d.set_ydata(intensity)
                        print("hey")

                    elif self.commun_state == LIVEIMU:
                        self.robot.update_robot_from_reception(received_data[0:len(received_data)-2])
                        self.liveIMU.addValue(received_data[len(received_data)-1])

                        time_l, intensity = self.liveIMU.get_values()
                        self.line_live_IMU.set_xdata(time_l)
                        self.line_live_IMU.set_ydata(intensity)
                else:
                    print("Not good data")
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)
            print(self.commun_state)

    #disables the continuous reading
    #and enables the continuous sending and receiving
    def setContSendAndReceive(self):
        self.commun_state = CONTROLANDREAD
        

    #enables the continuous reading
    #and disables the continuous sending and receiving
    def setContReceiveCaptorD(self, line_capt_d):
        self.commun_state = DCCALIBRATION

        self.line_capt_d = line_capt_d
        self.captorD = CaptorDist()
        
    def setContLiveIMU(self, line_live_IMU):
        self.commun_state = LIVEIMU

        self.line_live_IMU = line_live_IMU
        self.liveIMU = LiveIMU()

    #disables the continuous reading
    #and disables the continuous sending and receiving
    def stop_reading(self):
        self.commun_state = IDLE

    def check_length(self,length):
        if self.commun_state == CONTROLANDREAD:
             return length == 3 + len(self.robot.capteurs)

        elif self.commun_state == DCCALIBRATION:
            return length == 2

        elif self.commun_state == LIVEIMU:
             return length == 3 + len(self.robot.capteurs) + 1

        else:
            return False


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

    def buffer_clean_opti(self):
        if self.port.inWaiting() > 2000:
            self.port.read(self.port.inWaiting())
