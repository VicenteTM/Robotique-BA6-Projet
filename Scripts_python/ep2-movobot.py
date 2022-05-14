# EP2-MOVOBOT
# Main Movobot program
# Uses matplotlib as GUI and pyserial for communication


# Libraries import
import sys
import os
import matplotlib
from platform import release
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

# Project libraries import
import movobot_lib.communication as communication
from movobot_lib.robotPlot import Robot, ROBOT_DIAMETER, SIZEFROMROBOT
from movobot_lib.communication import serial_thread
from movobot_lib.plotUtilities import goodbye
from movobot_lib.captorsPlot import TIME_BACK_IMU


# Saves the calibration of the captors into lists in a python files
def saveCalibrationCallback(val):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        script_foleder = os.path.dirname(os.path.realpath(__file__))
        with open(script_foleder + '/calibrations/calibration.py', 'w') as doc:
            doc.write('Distance = [')
            for dist in reader_thd.captorD.dist:
                doc.write(f'{"%.0f" % dist}, ')
            doc.write('] \n\n')

            doc.write('Intensity = [')
            for inten in reader_thd.captorD.intensity:
                doc.write(f'{inten}, ')
            doc.write('] \n')
        robot.calibrated = True


# Button Control and Read callback function
def controlAndReadCallback(val):

    # Close all other windows
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
    elif reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

    reader_thd.setContControlAndRead()  # Set the state


# Button Plot Calibration callback function
def plotDCCallback(val):

    # Close LiveIMU window 
    if reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

    # Creates window if not opened // close it otherwise
    if reader_thd.commun_state != communication.DCCALIBRATION:

        # Window definition
        global fig_plotDCaptor
        fig_plotDCaptor, ax_plotDCaptor = plt.subplots()
        line_capt_d, = ax_plotDCaptor.plot([], [], '-r')
        
        # Plot definitions
        plt.title("E-Puck2 proximity sensor calibration")
        plt.xlabel("Distance (in mm)")
        plt.ylabel("Intensity")
        ax_plotDCaptor.set_xlim([0, 50])
        ax_plotDCaptor.set_ylim([0, 4000])

        # Config of the button
        colorAx             = 'lightgoldenrodyellow'
        saveCalibrationAx    = plt.axes([0.1, 0.02, 0.15, 0.04],figure=fig_plotDCaptor)
        saveCalibrationButton    = Button(saveCalibrationAx, 'Save', color=colorAx, hovercolor='0.975')

        # Connect User interactions
        fig_plotDCaptor.canvas.mpl_connect('key_press_event', on_press)
        fig_plotDCaptor.canvas.mpl_connect('key_release_event', release)
        fig_plotDCaptor.canvas.mpl_connect('close_event', handle_close_plotDC) #to detect when the window is closed and if we do a ctrl-c
        saveCalibrationButton.on_clicked(saveCalibrationCallback)
        fig_plotDCaptor._my_btn = saveCalibrationButton

        reader_thd.setContReceiveCaptorD(line_capt_d)  # Set the state

        plt.show()
    else:   # Re-click --> close window
        plt.close(fig=fig_plotDCaptor)
        reader_thd.stop_reading()   # Set the state

        
# Button Plot LiveIMU callback function
def plotLiveIMUCallback(val):

    # Close Calibration window 
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
        
    # Creates window if not opened // close it otherwise
    if reader_thd.commun_state != communication.LIVEIMU:

        # Window definition
        global fig_plotLiveIMU
        fig_plotLiveIMU, ax_plotLiveIMU = plt.subplots()
        line_live_IMU, = ax_plotLiveIMU.plot([], [], '-g')

        # Plot definitions
        ax_plotLiveIMU.set_xlim([TIME_BACK_IMU, 0])
        ax_plotLiveIMU.set_ylim([-20, 20])
        plt.title("Live IMU")
        plt.xlabel("Time (in seconds)")
        plt.ylabel("Y accelereration (in m/s^2)")

        # Connect User interactions
        fig_plotLiveIMU.canvas.mpl_connect('key_press_event', on_press)
        fig_plotLiveIMU.canvas.mpl_connect('key_release_event', release)
        fig_plotLiveIMU.canvas.mpl_connect('close_event', handle_close_plotLIMU) #to detect when the window is closed and if we do a ctrl-c

        reader_thd.setContLiveIMU(line_live_IMU)  # Set the state

        plt.show()
    else:   # Re-click --> close window
        plt.close(fig=fig_plotLiveIMU)
        reader_thd.setContControlAndRead()   # Set the state


# Button Stop callback function
def stop_readingCallback(val):
    # Close all windows but principal window
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
    elif reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

    reader_thd.stop_reading()  # Set the state


# Key event on press callback
def on_press(event):
    if event.key == 'up':
        robot.command = communication.FORWARD
    elif event.key == 'down':
        robot.command = communication.BACKWARD
    elif event.key == 'left':
        robot.command = communication.LEFT
    elif event.key == 'right':
        robot.command = communication.RIGHT


# Key event on release callback
def release(event):
    if event.key == 'up':
        robot.command = communication.NEUTRAL
    elif event.key == 'down':
        robot.command = communication.NEUTRAL
    elif event.key == 'left':
        robot.command = communication.NEUTRAL
    elif event.key == 'right':
        robot.command = communication.NEUTRAL


# Handler when closing the principal window
def handle_close(evt):

    # Close all windows 
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
    if reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

    # Stop the serial thread
    reader_thd.stop()
    print(goodbye)


# Handler when closing the calibration window
def handle_close_plotDC(evt):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)

        reader_thd.stop_reading()   # Set the state
        update_plot()


# Handler when closing the LiveIMU window
def handle_close_plotLIMU(evt):
    if reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

        reader_thd.setContControlAndRead() # Set the state


# Update the plots periodicaly
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig_r.canvas.draw_idle()
        reader_thd.plot_updated()
    if reader_thd.commun_state == communication.DCCALIBRATION:
        fig_plotDCaptor.canvas.draw() 
    if reader_thd.commun_state == communication.LIVEIMU:
        fig_plotLiveIMU.canvas.draw() 
    

#reset the robot position and clear captor drawing on the plot
def reset(event):
    robot.reset()


# Main window creation
def plotMovobot(fig_r, ax_r):

    # Window definition
    mng = plt.get_current_fig_manager()
    mng.window.resizable(False, False)
    mng.window.wm_geometry("+0+0")
    ax_r.set_aspect('equal', adjustable='box')

    # Plot definitions
    ax_r.set_xlim([-SIZEFROMROBOT + robot.position.getx(), SIZEFROMROBOT + robot.position.getx()])
    ax_r.set_ylim([-SIZEFROMROBOT + robot.position.gety(), SIZEFROMROBOT + robot.position.gety()])
    plt.title("Movobot position and obstacles")
    plt.xlabel("X (in mm)")
    plt.ylabel("Y (in mm)")
    plt.grid(color = 'green', linestyle = '--', linewidth = 0.5)
    

    # Timer to update the plot from within the state machine of matplotlib
    # Because matplotlib is not thread safe...
    timer = fig_r.canvas.new_timer(interval=50)
    timer.add_callback(update_plot)
    timer.start()

    # Positions of the buttons
    colorAx             = 'lightgoldenrodyellow'
    resetAx             = plt.axes([0.8 , 0.01, 0.1, 0.04] ,figure=fig_r)
    sendAndReceiveAx    = plt.axes([0.1 , 0.01, 0.15, 0.04],figure=fig_r)
    plotCaptorDistAx    = plt.axes([0.25, 0.01, 0.1, 0.04] ,figure=fig_r)
    plotLiveIMUAx       = plt.axes([0.35, 0.01, 0.1, 0.04] ,figure=fig_r)
    stopAx              = plt.axes([0.45, 0.01, 0.1, 0.04] ,figure=fig_r)
    stateAx              = plt.axes([0.15, 0.5, 0, 0] ,figure=fig_r)

    # Config of the buttons
    resetButton             = Button(resetAx, 'Reset Plot', color=colorAx, hovercolor='0.975')
    sendAndReceiveButton    = Button(sendAndReceiveAx, 'Control and read', color=colorAx, hovercolor='0.975')
    captorDistButton        = Button(plotCaptorDistAx, 'Calib Dist Captor', color=colorAx, hovercolor='0.975')
    captorLIMUButton        = Button(plotLiveIMUAx, 'Live IMU', color=colorAx, hovercolor='0.975')
    stop                    = Button(stopAx, 'Stop', color=colorAx, hovercolor='0.975')

    # Current State Box
    stateAx.get_xaxis().set_visible(False)
    stateAx.get_yaxis().set_visible(False)
    state_t = stateAx.text(0,0, 
                                f'Current state: \nIDLE \n\nCalibration:\nNot calibrated ', style = 'normal', ha = 'center',
                                bbox={'facecolor':'red', 'alpha':0.5, 'pad':10})
    reader_thd.set_state_t(state_t)

    # Connect User interactions
    resetButton.on_clicked(reset)
    sendAndReceiveButton.on_clicked(controlAndReadCallback)
    captorDistButton.on_clicked(plotDCCallback)
    captorLIMUButton.on_clicked(plotLiveIMUCallback)
    stop.on_clicked(stop_readingCallback)
    fig_r.canvas.mpl_connect('key_press_event', on_press)
    fig_r.canvas.mpl_connect('key_release_event', release)
    fig_r.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c

    plt.show()


def main():
    # Test if the serial port as been given as argument in the terminal or in text file
    if not len(sys.argv) == 1:
        port = sys.argv[1]
    else:
        try:
            with open('WRITE_PORT_HERE.txt') as doc:
                port = doc.read()
        except FileNotFoundError:
            port=''
        if port == '':
            print('Please give the serial port to use as argument')
            sys.exit(0)

    # Principal window plot declaration
    global fig_r
    fig_r, ax_r = plt.subplots(figsize=(15.36, 7.624))

    # Robot instance declaration
    global robot
    robot = Robot(fig_r, ax_r)
    robot.command = communication.NEUTRAL

    # Serial reader thread config
    # Begins the serial thread
    global reader_thd
    reader_thd = serial_thread(port , robot)
    reader_thd.start()

    plotMovobot(fig_r, ax_r)


if __name__ == "__main__":
    main()

