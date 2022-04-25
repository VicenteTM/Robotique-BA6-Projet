import sys
from threading import Thread
import serial
import time
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from robotPlot import Robot
from communication import readFloatSerial, sendFloatSerial
from plotUtilities import goodbye


def on_press(event):
    if event.key == 'up':
        robot.move(d_r = 1)
        robot.command = 0
    elif event.key == 'down':
        robot.move(d_r = -1)
        robot.command = 1
    elif event.key == 'left':
        robot.move(d_theta = 10)
        robot.command = 2
    elif event.key == 'right':
        robot.move(d_theta = -10)
        robot.command = 3


#handler when closing the window
def handle_close(evt):
    # we stop the serial thread
    reader_thd.stop()
    print(goodbye)

#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig.canvas.draw_idle()
        reader_thd.plot_updated()

#reset the sinus plot
def reset(event):
    return

def readcommand(port):
    command = readFloatSerial(port) 
    if(len(command)>0):
        print(command)

# #thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = False
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
                sendFloatSerial(self.port,robot.command)
                # readcommand(self.port)

            elif(self.contReceive):
                readcommand(self.port)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    #and disables the continuous sending and receiving
    def setContReceive(self, val):  
        self.contSendAndReceive = False
        self.contReceive = True

    #disables the continuous reading
    #and enables the continuous sending and receiving
    def setContSendAndReceive(self, val):
        self.contSendAndReceive = True
        self.contReceive = False

    #disables the continuous reading
    #and disables the continuous sending and receiving
    def stop_reading(self, val):
        self.contSendAndReceive = False
        self.contReceive = False

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

        
# #test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)
    

# #serial reader thread config
# #begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()

fig, ax = plt.subplots(figsize=(19,9))

fig.canvas.mpl_connect('key_press_event', on_press)
fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c
mng = plt.get_current_fig_manager()
mng.window.resizable(False, False)
mng.window.wm_geometry("+0+0")

#timer to update the plot from within the state machine of matplotlib
#because matplotlib is not thread safe...
timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

robot = Robot(fig, ax)

#positions of the buttons, sliders and radio buttons
colorAx             = 'lightgoldenrodyellow'
resetAx             = plt.axes([0.8, 0.025, 0.1, 0.04])
sendAndReceiveAx    = plt.axes([0.1, 0.025, 0.15, 0.04])
receiveAx           = plt.axes([0.25, 0.025, 0.1, 0.04])
stopAx              = plt.axes([0.35, 0.025, 0.1, 0.04])

#config of the buttons, sliders and radio buttons
resetButton             = Button(resetAx, 'Reset sinus', color=colorAx, hovercolor='0.975')
sendAndReceiveButton    = Button(sendAndReceiveAx, 'Control and read', color=colorAx, hovercolor='0.975')
receiveButton           = Button(receiveAx, 'Only control', color=colorAx, hovercolor='0.975')
stop                    = Button(stopAx, 'Stop', color=colorAx, hovercolor='0.975')

ax.set_xlim([-5, 5])
ax.set_ylim([-5, 5])
ax.set_aspect('equal', adjustable='box')

resetButton.on_clicked(reset)
sendAndReceiveButton.on_clicked(reader_thd.setContSendAndReceive)
receiveButton.on_clicked(reader_thd.setContReceive)
stop.on_clicked(reader_thd.stop_reading)

plt.show()
