from platform import release
import sys
import time
import matplotlib

from plotDCaptor import plotDistCaptor

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

from robotPlot import Robot, robot_diameter
from communication import serial_thread
import communication
from plotUtilities import goodbye


def on_press(event):
    if event.key == 'up':
        robot.move(d_r = 1)
        robot.command = communication.FORWARD
    elif event.key == 'down':
        robot.move(d_r = -1)
        robot.command = communication.BACKWARD
    elif event.key == 'left':
        robot.move(d_theta = 10)
        robot.command = communication.LEFT
    elif event.key == 'right':
        robot.move(d_theta = -10)
        robot.command = communication.RIGHT
    elif event.key == ' ':
        robot.command = communication.NEUTRAL

def release(event):
    if event.key == 'up':
        robot.command = communication.NEUTRAL
    elif event.key == 'down':
        robot.command = communication.NEUTRAL
    elif event.key == 'left':
        robot.command = communication.NEUTRAL
    elif event.key == 'right':
        robot.command = communication.NEUTRAL


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

def plotMovobot(fig, ax):
    fig.canvas.mpl_connect('key_press_event', on_press)
    fig.canvas.mpl_connect('key_release_event', release)
    fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c
    mng = plt.get_current_fig_manager()
    mng.window.resizable(False, False)
    mng.window.wm_geometry("+0+0")

    #timer to update the plot from within the state machine of matplotlib
    #because matplotlib is not thread safe...
    timer = fig.canvas.new_timer(interval=50)
    timer.add_callback(update_plot)
    timer.start()


    #positions of the buttons, sliders and radio buttons
    colorAx             = 'lightgoldenrodyellow'
    resetAx             = plt.axes([0.8, 0.025, 0.1, 0.04])
    sendAndReceiveAx    = plt.axes([0.1, 0.025, 0.15, 0.04])
    ploCaptorDist       = plt.axes([0.25, 0.025, 0.1, 0.04])
    stopAx              = plt.axes([0.35, 0.025, 0.1, 0.04])

    #config of the buttons, sliders and radio buttons
    resetButton             = Button(resetAx, 'Reset sinus', color=colorAx, hovercolor='0.975')
    sendAndReceiveButton    = Button(sendAndReceiveAx, 'Control and read', color=colorAx, hovercolor='0.975')
    captorDistButton        = Button(ploCaptorDist, 'Plot Dist Captor', color=colorAx, hovercolor='0.975')
    stop                    = Button(stopAx, 'Stop', color=colorAx, hovercolor='0.975')

    sizefromrobot = 10 * robot_diameter

    ax.set_xlim([-sizefromrobot, sizefromrobot])
    ax.set_ylim([-sizefromrobot, sizefromrobot])
    ax.set_aspect('equal', adjustable='box')

    resetButton.on_clicked(reset)
    sendAndReceiveButton.on_clicked(reader_thd.setContSendAndReceive)
    captorDistButton.on_clicked(reader_thd.setContPlotDCaptor)
    stop.on_clicked(reader_thd.stop_reading)
    plt.show()           


def main():
    # #test if the serial port as been given as argument in the terminal
    if len(sys.argv) == 1:
        print('Please give the serial port to use as argument')
        sys.exit(0)

    global fig
    fig, ax = plt.subplots(figsize=(19,9))

    global robot
    robot = Robot(fig, ax)
    robot.command = communication.NEUTRAL

    # #serial reader thread config
    # #begins the serial thread
    global reader_thd
    reader_thd = serial_thread(sys.argv[1], robot, fig, ax)
    reader_thd.start()

    plotMovobot(fig, ax)

if __name__ == "__main__":
    main()
