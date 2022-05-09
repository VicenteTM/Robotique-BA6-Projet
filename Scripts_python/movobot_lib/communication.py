# EP2-MOVOBOT
# File resposnsible for communication and reference of the state of the program


# Libraries import
import sys
import struct
from threading import Thread
import time
import numpy as np
import serial
from movobot_lib.captorsPlot import CaptorDist, LiveIMU
from movobot_lib.robotPlot import Robot

# Max size of buffer before overloadind (in bytes)
BUFFEROVERLOAD = 2000

# Robot Commands
NEUTRAL = 0
FORWARD = 1
BACKWARD = 2
LEFT = 3
RIGHT = 4

# Program States
IDLE = 0
CONTROLANDREAD = 1
DCCALIBRATION = 2
LIVEIMU = 3

# Sending function to rhe robot
def sendRobotData(port,data_to_send):

    # Convert the sending data into numpy
    data = np.array(data_to_send).astype(np.int16)
    size = np.array([data.size], dtype=np.int16)

    send_buffer = bytearray([])

    # Add the data in the sending buffer
    for i in range(0,size[0]):
        send_buffer += struct.pack('<h',data[i])

    # Set to send: "START" (flag) + size of the following data + data
    port.write(b'START')
    port.write(struct.pack('<h',2*size[0]))
    port.write(send_buffer)
    print('sent !')

# Reading function from the robot to the program: set to read in Uint16
# Set to receive: "START" (flag) + size of the following data + data
def readUInt16Serial(port):

    # Flag detection loop
    state = 0
    while(state != 5):
        
        # Reads 1 byte
        c1 = port.read(1)

        # Timeout condition
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

    # Reads the size
    # Converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    # Removes the second element which is void
    size = size[0]

    # Reads the data
    rcv_buffer = port.read(size*2)

    # If the wrong length is received --> Timeout
    if not len(rcv_buffer) == 2*size:
        print('Timout...')
        return []

    # Read the data and put it in a list
    data = []
    for i in range(0,size):
        data.append(struct.unpack_from('<h',rcv_buffer, i*2)[0])

    print('received !')
    return data

# Function that pre-evaluates the value received (not full use needed) 
def readfromrobot(port):
    value = readUInt16Serial(port)
    return value

# Thread: send, receive to and from the robot and responsible for the interface with the user
class serial_thread(Thread):

    # Init function called when the thread begins
    def __init__(self, port, robot):
        Thread.__init__(self)
        self.robot = robot
        self.alive = True
        self.need_to_update = False
        self.commun_state = IDLE

        print('Connecting to port {}'.format(port))
        
        # Check if the port is available
        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)

    # While loop of the thread
    def run(self):
        while(self.alive):
            sendRobotData(self.port, [self.commun_state,self.robot.command])

            # If the state is not an active state
            if not self.commun_state in [CONTROLANDREAD, DCCALIBRATION, LIVEIMU]:
                # Flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)
                continue
                
            self.buffer_clean_opti()        # Buffer optimisation
            received_data = readfromrobot(self.port)    # Receive data from robot

            if not self.good_length(received_data):
                continue

            if self.commun_state == CONTROLANDREAD:
                self.controlAndReadAction(received_data)

            elif self.commun_state == DCCALIBRATION:
                self.dcCalibrationAction(received_data)

            elif self.commun_state == LIVEIMU:
                self.liveImuAction(received_data)

    def controlAndReadAction(self,received_data):
        self.robot.update_robot_from_reception(received_data)

    def dcCalibrationAction(self, received_data):
        self.captorD.addValues(received_data)
        dist, intensity = self.captorD.get_values()

        self.line_capt_d.set_xdata(dist)
        self.line_capt_d.set_ydata(intensity)

    def liveImuAction(self, received_data):
        self.robot.update_robot_from_reception(received_data[0:len(received_data)-2])
        self.liveIMU.addValue(received_data[len(received_data)-1])

        time_l, intensity = self.liveIMU.get_values()
        self.line_live_IMU.set_xdata(time_l)
        self.line_live_IMU.set_ydata(intensity)

    def good_length(self,data):
        length = len(data)
        if self.commun_state == CONTROLANDREAD:
            return length == 3 + len(self.robot.capteurs) # X+ Y + direction + Captors

        elif self.commun_state == DCCALIBRATION:
            return length == 2  # Distance + Intensity

        elif self.commun_state == LIVEIMU:
            return length == 3 + len(self.robot.capteurs) + 1 # X+ Y + direction + Captors + IMU

        return False

    # Cleans the buffer if it is overloaded
    def buffer_clean_opti(self):
        if self.port.inWaiting() > BUFFEROVERLOAD:
            self.port.read(self.port.inWaiting())
    
    # Set Control and read state and the control of the robot and reception of data
    def setContControlAndRead(self):
        self.commun_state = CONTROLANDREAD
        
    # Set the Calibration state
    def setContReceiveCaptorD(self, line_capt_d):
        self.commun_state = DCCALIBRATION

        self.line_capt_d = line_capt_d
        self.captorD = CaptorDist()
        
    # Set LiveIMU state and the control of the robot and reception of data
    def setContLiveIMU(self, line_live_IMU):
        self.commun_state = LIVEIMU

        self.line_live_IMU = line_live_IMU
        self.liveIMU = LiveIMU()

    # Disables the continuous reading and sending
    def stop_reading(self):
        self.commun_state = IDLE


    # Tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    # Tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    # Tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    # Clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()
