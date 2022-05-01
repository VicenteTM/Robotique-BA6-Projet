import sys
import matplotlib
from platform import release
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from robotPlot import Robot, robot_diameter
from communication import serial_thread
import communication
from plotUtilities import goodbye

def sendAndReceiveCallback(val):
    reader_thd.setContSendAndReceive()

def stop_readingCallback(val):
    reader_thd.stop_reading()

def plotDCCallback(val):
    global DCaptor_on
    if DCaptor_on is False:
        global fig_plotDCaptor
        fig_plotDCaptor, ax_plotDCaptor = plt.subplots()
        fig_plotDCaptor.canvas.mpl_connect('key_press_event', on_press)
        fig_plotDCaptor.canvas.mpl_connect('key_release_event', release)
        fig_plotDCaptor.canvas.mpl_connect('close_event', handle_close_plot) #to detect when the window is closed and if we do a ctrl-c
        ax_plotDCaptor.set_xlim([0, 5])
        ax_plotDCaptor.set_ylim([0, 4000])
        line_capt_d, = ax_plotDCaptor.plot([], [], '-r')
        reader_thd.setContReceiveCaptorD(line_capt_d)
        plt.show()
        DCaptor_on = True
    else:
        plt.close(fig=fig_plotDCaptor)
        DCaptor_on = False


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

def handle_close_plot(evt):
    reader_thd.stop_reading()
    plt.close(fig=fig_plotDCaptor)
    global DCaptor_on
    DCaptor_on = False


#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig_r.canvas.draw_idle()
        reader_thd.plot_updated()
    if DCaptor_on:
        fig_plotDCaptor.canvas.draw()
        fig_plotDCaptor.canvas.flush_events()
    

#reset the sinus plot
def reset(event):
    return


def plotMovobot(fig_r, ax_r):
    fig_r.canvas.mpl_connect('key_press_event', on_press)
    fig_r.canvas.mpl_connect('key_release_event', release)
    fig_r.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c
    mng = plt.get_current_fig_manager()
    mng.window.resizable(False, False)
    mng.window.wm_geometry("+0+0")

    #timer to update the plot from within the state machine of matplotlib
    #because matplotlib is not thread safe...
    timer = fig_r.canvas.new_timer(interval=50)
    timer.add_callback(update_plot)
    timer.start()


    #positions of the buttons, sliders and radio buttons
    colorAx             = 'lightgoldenrodyellow'
    resetAx             = plt.axes([0.8, 0.025, 0.1, 0.04],figure=fig_r)
    sendAndReceiveAx    = plt.axes([0.1, 0.025, 0.15, 0.04],figure=fig_r)
    plotCaptorDistAx    = plt.axes([0.25, 0.025, 0.1, 0.04],figure=fig_r)
    stopAx              = plt.axes([0.35, 0.025, 0.1, 0.04],figure=fig_r)

    #config of the buttons, sliders and radio buttons
    resetButton             = Button(resetAx, 'Reset sinus', color=colorAx, hovercolor='0.975')
    sendAndReceiveButton    = Button(sendAndReceiveAx, 'Control and read', color=colorAx, hovercolor='0.975')
    captorDistButton        = Button(plotCaptorDistAx, 'Plot Dist Captor', color=colorAx, hovercolor='0.975')
    stop                    = Button(stopAx, 'Stop', color=colorAx, hovercolor='0.975')

    ax_r.set_aspect('equal', adjustable='box')

    resetButton.on_clicked(reset)
    sendAndReceiveButton.on_clicked(sendAndReceiveCallback)
    captorDistButton.on_clicked(plotDCCallback)
    stop.on_clicked(stop_readingCallback)

    
    sizefromrobot = 10 * robot_diameter

    ax_r.set_xlim([-sizefromrobot, sizefromrobot])
    ax_r.set_ylim([-sizefromrobot, sizefromrobot])

    plt.show()


def main():
    # #test if the serial port as been given as argument in the terminal
    if len(sys.argv) == 1:
        print('Please give the serial port to use as argument')
        sys.exit(0)
    
    global DCaptor_on
    DCaptor_on = False
    global fig_r
    fig_r, ax_r = plt.subplots(figsize=(19,9))

    global robot
    robot = Robot(fig_r, ax_r)
    robot.command = communication.NEUTRAL

    # #serial reader thread config
    # #begins the serial thread
    global reader_thd
    reader_thd = serial_thread(sys.argv[1], robot)
    reader_thd.start()

    plotMovobot(fig_r, ax_r)


if __name__ == "__main__":
    main()

