"""
==============
Keypress event
==============

Show how to connect to keypress events.
"""

import struct
import sys
from threading import Thread
import serial
import time
import numpy as np
import matplotlib.pyplot as plt

import math


#Can be converted into a portable package by using the PyInstaller module
# pip install pyinstaller (need to be used with Python3)
# cf. https://pyinstaller.readthedocs.io/en/v3.3.1/usage.html

goodbye = """
          |\      _,,,---,,_
          /,`.-'`'    -.  ;-;;,_
         |,4-  ) )-,_..;\ (  `'-'
 _______'---''(_/--'__`-'\_)______   ______            _______  _
(  ____ \(  ___  )(  ___  )(  __  \ (  ___ \ |\     /|(  ____ \| |
| (    \/| (   ) || (   ) || (  \  )| (   ) )( \   / )| (    \/| |
| |      | |   | || |   | || |   ) || (__/ /  \ (_) / | (__    | |
| | ____ | |   | || |   | || |   | ||  __ (    \   /  |  __)   | |
| | \_  )| |   | || |   | || |   ) || (  \ \    ) (   | (      |_|
| (___) || (___) || (___) || (__/  )| )___) )   | |   | (____/\ _ 
(_______)(_______)(_______)(______/ |______/    \_/   (_______/(_)                                         
"""

goodbye2 = """
                   /\_/\\
                 =( °w° )=
                   )   (  //
                  (__ __)//
 _____                 _ _                _ 
|  __ \               | | |              | |
| |  \/ ___   ___   __| | |__  _   _  ___| |
| | __ / _ \ / _ \ / _` | '_ \| | | |/ _ \ |
| |_\ \ (_) | (_) | (_| | |_) | |_| |  __/_|
 \____/\___/ \___/ \__,_|_.__/ \__, |\___(_)
                                __/ |       
                               |___/        
"""

#Robot constants
distance_captor_max_range = 0.4
wheel_dist = 1
wheel_diameter = 1
robot_diameter = 5
wheel_radius = wheel_diameter/2
robot_radius = robot_diameter/2

def rect(r, theta):
    """theta in degrees

    returns tuple; (float, float); (x,y)
    """
    x = r * math.cos(math.radians(theta))
    y = r * math.sin(math.radians(theta))
    return x,y


def polar(x, y):
    """returns r, theta(degrees)
    """
    r = (x ** 2 + y ** 2) ** .5
    theta = math.degrees(math.atan2(y,x))
    return r, theta


class Vector(object):
    def __init__(self, x=None, y=None, r=None, theta=None):
        """x and y or r and theta(degrees)
        """
        if x is not None and y is not None:
            self.c_polar(x, y)
        elif r is not None and theta is not None:
            self.c_rect(r, theta)
        else:
            raise ValueError('Must specify x and y or r and theta')
    def c_polar(self, x, y, f = polar):
        self._x = x
        self._y = y
        self._r, self._theta = f(self._x, self._y)
        self._theta_radians = math.radians(self._theta)
    def c_rect(self, r, theta, f = rect):
        """theta in degrees
        """
        self._r = r
        self._theta = theta
        self._theta_radians = math.radians(theta)
        self._x, self._y = f(self._r, self._theta)
    def setx(self, x):
        self.c_polar(x, self._y)
    def getx(self):
        return self._x
    x = property(fget = getx, fset = setx)
    def sety(self, y):
        self.c_polar(self._x, y)
    def gety(self):
        return self._y
    y = property(fget = gety, fset = sety)
    def setxy(self, x, y):
        self.c_polar(x, y)
    def getxy(self):
        return self._x, self._y
    xy = property(fget = getxy, fset = setxy)
    def setr(self, r):
        self.c_rect(r, self._theta)
    def getr(self):
        return self._r
    r = property(fget = getr, fset = setr)
    def settheta(self, theta):
        """theta in degrees
        """
        self.c_rect(self._r, theta)
    def gettheta(self):
        return self._theta
    theta = property(fget = gettheta, fset = settheta)
    def set_r_theta(self, r, theta):
        """theta in degrees
        """
        self.c_rect(r, theta)
    def get_r_theta(self):
        return self._r, self._theta
    r_theta = property(fget = get_r_theta, fset = set_r_theta)
    def __str__(self):
        return '({},{})'.format(self._r, self._theta)
    def __add__(self, other):
        sum_x = self._x + other.x
        sum_y = self._y + other.y
        return Vector(x = sum_x, y = sum_y)
    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)


class Capteur_dist:
    def __init__(self,position,direction,angle_rel):
        self.position = position
        self.direction = direction
        self.angle_rel = angle_rel
        self.direction.r = distance_captor_max_range
        self.arrow = plt.arrow(self.position.x, self.position.y, self.direction.x, self.direction.y, length_includes_head = True, width = 0.01)

    def update(self,position,direction):
        self.position = position
        self.direction = direction
        self.direction.r = distance_captor_max_range
        self.arrow.set_data(x=self.position.x, y=self.position.y, dx=self.direction.x, dy=self.direction.y)


class Robot:

    def __init__(self, position,direction,capteur_angles):
        self.bo = True
        self.position = position
        self.direction = Vector(r=1,theta=direction)

        self.capteurs = []
        for capteur_angle in capteur_angles:
            position_c_abs,position_c_rel = self.compute_capteur(capteur_angle)
            capteur = Capteur_dist(position_c_abs,position_c_rel,capteur_angle)
            self.capteurs.append(capteur)

        self.circle = plt.Circle(self.position.xy, robot_radius,fill=True, ec= 'black',fc= 'white')
        ax.add_artist(self.circle)

        x_arrow, y_arrow, dx_arrow, dy_arrow = self.compute_arrow()
        self.arrow = plt.arrow(x_arrow, y_arrow, dx_arrow, dy_arrow,length_includes_head = True, width = 0.1*robot_radius)

        x_right_data, y_right_data = self.compute_wheel_position('right')
        self.right_wheel = plt.Line2D(x_right_data, y_right_data)

        x_left_data, y_left_data = self.compute_wheel_position('left')
        self.left_wheel = plt.Line2D(x_left_data, y_left_data)
        ax.add_line(self.right_wheel)
        ax.add_line(self.left_wheel)

    def compute_capteur(self,capteur_angle):
        position_c_rel = Vector(r = robot_radius, theta = capteur_angle + self.direction.theta)
        position_c_abs = self.position + position_c_rel
        return position_c_abs, position_c_rel

    def compute_arrow(self):
        self.direction.r = 1
        x_arrow = self.position.x-robot_radius*self.direction.x
        y_arrow = self.position.y-robot_radius*self.direction.y
        dx_arrow =  robot_diameter*(self.direction.x)
        dy_arrow =  robot_diameter*(self.direction.y)

        return x_arrow, y_arrow, dx_arrow, dy_arrow

    def compute_wheel_position(self,side):
        if side == 'right':
            alpha = -90
        elif side == 'left':
            alpha = 90
        else:
            return None

        center_wheel = self.position + Vector(r=wheel_dist/2,theta=self.direction.theta + alpha)
        min_wheel = center_wheel + Vector(r=wheel_radius,theta=self.direction.theta)
        max_wheel = center_wheel + Vector(r=wheel_radius,theta=self.direction.theta + 180)
        x_data = [min_wheel.x, max_wheel.x]
        y_data = [min_wheel.y, max_wheel.y]
        return x_data, y_data

    def update_capteurs(self):
        for capteur in self.capteurs:
            position_c_rel,position_c_abs = self.compute_capteur(capteur.angle_rel)
            capteur.update(position_c_rel,position_c_abs)

    def update_robot_plot(self):
        self.circle.set(center=self.position.xy)
        x_arrow, y_arrow, dx_arrow, dy_arrow = self.compute_arrow()
        self.arrow.set_data(x = x_arrow, y = y_arrow,dx = dx_arrow, dy = dy_arrow)

        x_right_data, y_right_data = self.compute_wheel_position('right')
        x_left_data, y_left_data = self.compute_wheel_position('left')
        self.right_wheel.set(xdata = x_right_data, ydata = y_right_data)
        self.left_wheel.set(xdata = x_left_data, ydata = y_left_data)

    def move(self, d_r =None, d_theta=None):
        if d_r is not None:
            position_f_rel = Vector(r = d_r,theta=self.direction.theta)
            self.position = self.position + position_f_rel
            self.update_capteurs()
            self.update_robot_plot()
            fig.canvas.draw()
        elif d_theta is not None:
            self.direction.theta += d_theta
            self.update_capteurs()
            self.update_robot_plot()
            fig.canvas.draw()


def on_press(event):
    # print('press', event.key)
    # sys.stdout.flush()
    # if event.key == 'x':
    #     visible = xl.get_visible()
    #     xl.set_visible(not visible)
    #     fig.canvas.draw()
    if event.key == 'up':
        robot.move(d_r = 1)
    elif event.key == 'down':
        robot.move(d_r = -1)
    elif event.key == 'left':
        robot.move(d_theta = 10)
    elif event.key == 'right':
        robot.move(d_theta = -10)


# #handler when closing the window
# def handle_close(evt):
#     #we stop the serial thread
#     reader_thd.stop()
#     print(goodbye)

# #update the plots
# def update_plot():
#     if(reader_thd.need_to_update_plot()):
#         fig.canvas.draw_idle()
#         reader_thd.plot_updated()

# #sends the data of the sinus to the serial port in int16
# def sendFloatSerial(port):
#     data = (sinus_plot.get_ydata()).astype(np.int16)

#     #to convert to int16 we need to pass via numpy
#     size = np.array([data.size], dtype=np.int16)

#     send_buffer = bytearray([])

#     i = 0
#     while(i < size[0]):
#         send_buffer += struct.pack('<h',data[i])
#         i = i+1

#     port.write(b'START')
#     port.write(struct.pack('<h',2*size[0]))
#     port.write(send_buffer)
#     print('sent !')

# #reads the FFT in float32 from the serial
# def readFloatSerial(port):

#     state = 0

#     while(state != 5):

#         #reads 1 byte
#         c1 = port.read(1)
#         #timeout condition
#         if(c1 == b''):
#             print('Timout...')
#             return [];

#         if(state == 0):
#             if(c1 == b'S'):
#                 state = 1
#             else:
#                 state = 0
#         elif(state == 1):
#             if(c1 == b'T'):
#                 state = 2
#             elif(c1 == b'S'):
#                 state = 1
#             else:
#                 state = 0
#         elif(state == 2):
#             if(c1 == b'A'):
#                 state = 3
#             elif(c1 == b'S'):
#                 state = 1
#             else:
#                 state = 0
#         elif(state == 3):
#             if(c1 == b'R'):
#                 state = 4
#             elif (c1 == b'S'):
#                 state = 1
#             else:
#                 state = 0
#         elif(state == 4):
#             if(c1 == b'T'):
#                 state = 5
#             elif (c1 == b'S'):
#                 state = 1
#             else:
#                 state = 0

#     #reads the size
#     #converts as short int in little endian the two bytes read
#     size = struct.unpack('<h',port.read(2)) 
#     #removes the second element which is void
#     size = size[0]  

#     #reads the data
#     rcv_buffer = port.read(size*4)
#     data = []

#     #if we receive the good amount of data, we convert them in float32
#     if(len(rcv_buffer) == 4*size):
#         i = 0
#         while(i < size):
#             data.append(struct.unpack_from('<f',rcv_buffer, i*4))
#             i = i+1

#         print('received !')
#         return data
#     else:
#         print('Timout...')
#         return []

# #thread used to control the communication part
# class serial_thread(Thread):

#     #init function called when the thread begins
#     def __init__(self, port):
#         Thread.__init__(self)
#         self.contReceive = False
#         self.contSendAndReceive = False
#         self.alive = True
#         self.need_to_update = False

#         print('Connecting to port {}'.format(port))
        
#         try:
#             self.port = serial.Serial(port, timeout=0.5)
#         except:
#             print('Cannot connect to the e-puck2')
#             sys.exit(0)
#     #function called after the init
#     def run(self):
        
#         while(self.alive):
#             if(self.contSendAndReceive):
#                 sendFloatSerial(self.port)
#                 update_fft_plot(self.port)

#             elif(self.contReceive):
#                 update_fft_plot(self.port)
#             else:
#                 #flush the serial
#                 self.port.read(self.port.inWaiting())
#                 time.sleep(0.1)

#     #enables the continuous reading
#     #and disables the continuous sending and receiving
#     def setContReceive(self, val):  
#         self.contSendAndReceive = False
#         self.contReceive = True

#     #disables the continuous reading
#     #and enables the continuous sending and receiving
#     def setContSendAndReceive(self, val):
#         self.contSendAndReceive = True
#         self.contReceive = False

#     #disables the continuous reading
#     #and disables the continuous sending and receiving
#     def stop_reading(self, val):
#         self.contSendAndReceive = False
#         self.contReceive = False

#     #tell the plot need to be updated
#     def tell_to_update_plot(self):
#         self.need_to_update = True

#     #tell the plot has been updated
#     def plot_updated(self):
#         self.need_to_update = False

#     #tell if the plot need to be updated
#     def need_to_update_plot(self):
#         return self.need_to_update

#     #clean exit of the thread if we need to stop it
#     def stop(self):
#         self.alive = False
#         self.join()
#         if(self.port.isOpen()):
#             while(self.port.inWaiting() > 0):
#                 self.port.read(self.port.inWaiting())
#                 time.sleep(0.01)
#             self.port.close()

        
# #test if the serial port as been given as argument in the terminal
# if len(sys.argv) == 1:
#     print('Please give the serial port to use as argument')
#     sys.exit(0)
    

# #serial reader thread config
# #begins the serial thread
# reader_thd = serial_thread(sys.argv[1])
# reader_thd.start()


fig, ax = plt.subplots()

fig.canvas.mpl_connect('key_press_event', on_press)

# ax.plot(np.random.rand(12), np.random.rand(12), 'go', label = 'hey')
xl = ax.set_xlabel('easy come, easy go')
ax.set_title('Press a key')
robot = Robot(Vector(r=1,theta=-90),40,[0,45,-45,90,-90])
ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_aspect('equal', adjustable='box')
plt.show()
