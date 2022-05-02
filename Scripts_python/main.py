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
    
    global DCaptor_on
    global LiveIMU_on
    if DCaptor_on:
        plt.close(fig=fig_plotDCaptor)
        DCaptor_on = False
    elif LiveIMU_on:
        plt.close(fig=fig_plotLiveIMU)
        LiveIMU_on = False

    reader_thd.setContSendAndReceive()

def stop_readingCallback(val):
    global DCaptor_on
    global LiveIMU_on
    if DCaptor_on:
        plt.close(fig=fig_plotDCaptor)
        DCaptor_on = False
    elif LiveIMU_on:
        plt.close(fig=fig_plotLiveIMU)
        LiveIMU_on = False
    reader_thd.stop_reading()

def plotDCCallback(val):
    global LiveIMU_on
    if LiveIMU_on:
        plt.close(fig=fig_plotLiveIMU)
        LiveIMU_on = False

    global DCaptor_on
    if DCaptor_on is False:
        global fig_plotDCaptor
        fig_plotDCaptor, ax_plotDCaptor = plt.subplots()
        fig_plotDCaptor.canvas.mpl_connect('key_press_event', on_press)
        fig_plotDCaptor.canvas.mpl_connect('key_release_event', release)
        fig_plotDCaptor.canvas.mpl_connect('close_event', handle_close_plotDC) #to detect when the window is closed and if we do a ctrl-c
        ax_plotDCaptor.set_xlim([0, 2000])
        ax_plotDCaptor.set_ylim([0, 4000])
        line_capt_d, = ax_plotDCaptor.plot([], [], '-r')
        plt.title("E-Puck2 distance captor caracteristic")
        plt.xlabel("Distance (in mm)")
        plt.ylabel("Intensity")
        reader_thd.setContReceiveCaptorD(line_capt_d)
        plt.show()
        DCaptor_on = True
    else:
        plt.close(fig=fig_plotDCaptor)
        DCaptor_on = False

        
def plotLiveIMUCallback(val):
    global DCaptor_on
    if DCaptor_on:
        plt.close(fig=fig_plotDCaptor)
        DCaptor_on = False

    global LiveIMU_on
    if LiveIMU_on is False:
        global fig_plotLiveIMU
        fig_plotLiveIMU, ax_plotLiveIMU = plt.subplots()
        fig_plotLiveIMU.canvas.mpl_connect('key_press_event', on_press)
        fig_plotLiveIMU.canvas.mpl_connect('key_release_event', release)
        fig_plotLiveIMU.canvas.mpl_connect('close_event', handle_close_plotLIMU) #to detect when the window is closed and if we do a ctrl-c
        ax_plotLiveIMU.set_xlim([0, 50])
        ax_plotLiveIMU.set_ylim([0, 30])
        line_live_IMU, = ax_plotLiveIMU.plot([], [], '-g')
        plt.title("Live IMU")
        plt.xlabel("Live Time")
        plt.ylabel("Intensity")
        reader_thd.setContLiveIMU(line_live_IMU)
        plt.show()
        LiveIMU_on = True
    else:
        plt.close(fig=fig_plotLiveIMU)
        LiveIMU_on = False


def on_press(event):
    if event.key == 'up':
        robot.move(d_r = 20)
        robot.command = communication.FORWARD
    elif event.key == 'down':
        robot.move(d_r = -20)
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

def handle_close_plotDC(evt):
    plt.close(fig=fig_plotDCaptor)
    global DCaptor_on
    if DCaptor_on:
        reader_thd.stop_reading()
        DCaptor_on = False

def handle_close_plotLIMU(evt):
    plt.close(fig=fig_plotLiveIMU)
    global LiveIMU_on
    if LiveIMU_on:
        reader_thd.stop_reading()
        LiveIMU_on = False


#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig_r.canvas.draw_idle()
        reader_thd.plot_updated()
    if DCaptor_on:
        fig_plotDCaptor.canvas.draw()
        fig_plotDCaptor.canvas.flush_events()
    if LiveIMU_on:
        fig_plotLiveIMU.canvas.draw()
        fig_plotLiveIMU.canvas.flush_events()
    pass
    

#reset the sinus plot
def reset(event):
    robot.reset()


def plotMovobot(fig_r, ax_r):

    plt.title("E-Puck2 position")
    plt.xlabel("X (in mm)")
    plt.ylabel("Y (in mm)")
    
    plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)
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
    resetAx             = plt.axes([0.8, 0.02, 0.1, 0.04],figure=fig_r)
    sendAndReceiveAx    = plt.axes([0.1, 0.02, 0.15, 0.04],figure=fig_r)
    plotCaptorDistAx    = plt.axes([0.25, 0.02, 0.1, 0.04],figure=fig_r)
    plotLiveIMUAx       = plt.axes([0.35, 0.02, 0.1, 0.04],figure=fig_r)
    stopAx              = plt.axes([0.45, 0.02, 0.1, 0.04],figure=fig_r)

    #config of the buttons, sliders and radio buttons
    resetButton             = Button(resetAx, 'Reset Plot', color=colorAx, hovercolor='0.975')
    sendAndReceiveButton    = Button(sendAndReceiveAx, 'Control and read', color=colorAx, hovercolor='0.975')
    captorDistButton        = Button(plotCaptorDistAx, 'Plot Dist Captor', color=colorAx, hovercolor='0.975')
    captorLIMUButton        = Button(plotLiveIMUAx, 'Plot Live IMU', color=colorAx, hovercolor='0.975')
    stop                    = Button(stopAx, 'Stop', color=colorAx, hovercolor='0.975')

    ax_r.set_aspect('equal', adjustable='box')

    resetButton.on_clicked(reset)
    sendAndReceiveButton.on_clicked(sendAndReceiveCallback)
    captorDistButton.on_clicked(plotDCCallback)
    captorLIMUButton.on_clicked(plotLiveIMUCallback)
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
    global LiveIMU_on
    LiveIMU_on = False
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

