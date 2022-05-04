import sys
import matplotlib
from platform import release

import communication
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
from robotPlot import Robot, robot_diameter
from communication import serial_thread
import communication
from plotUtilities import goodbye
import os

def saveCalibrationCallback(val):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        script_foleder = os.path.dirname(os.path.realpath(__file__))
        with open(script_foleder + '/calibration.py', 'w') as doc:
            doc.write('Distance = [')
            for dist in reader_thd.captorD.dist:
                doc.write(f'{dist}, ')
            doc.write('] \n\n')

            doc.write('Intensity = [')
            for inten in reader_thd.captorD.intensity:
                doc.write(f'{inten}, ')
            doc.write('] \n')

def sendAndReceiveCallback(val):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
    elif reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

    reader_thd.setContSendAndReceive()

def stop_readingCallback(val):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
    elif reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)
    reader_thd.stop_reading()

def plotDCCallback(val):
    if reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)

    if reader_thd.commun_state != communication.DCCALIBRATION:
        global fig_plotDCaptor
        fig_plotDCaptor, ax_plotDCaptor = plt.subplots()
        fig_plotDCaptor.canvas.mpl_connect('key_press_event', on_press)
        fig_plotDCaptor.canvas.mpl_connect('key_release_event', release)
        fig_plotDCaptor.canvas.mpl_connect('close_event', handle_close_plotDC) #to detect when the window is closed and if we do a ctrl-c
        ax_plotDCaptor.set_xlim([0, 2000])
        ax_plotDCaptor.set_ylim([0, 4000])
        plt.title("E-Puck2 distance captor caracteristic")
        plt.xlabel("Distance (in mm)")
        plt.ylabel("Intensity")

        
        colorAx             = 'lightgoldenrodyellow'
        saveCalibrationAx    = plt.axes([0.1, 0.02, 0.15, 0.04],figure=fig_plotDCaptor)
        saveCalibrationButton    = Button(saveCalibrationAx, 'Save', color=colorAx, hovercolor='0.975')
        saveCalibrationButton.on_clicked(saveCalibrationCallback)

        line_capt_d, = ax_plotDCaptor.plot([], [], '-r')
        reader_thd.setContReceiveCaptorD(line_capt_d)
        plt.show()
        fig_plotDCaptor._my_btn = saveCalibrationButton
    else:
        plt.close(fig=fig_plotDCaptor)
        reader_thd.stop_reading()

        
def plotLiveIMUCallback(val):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)

    if reader_thd.commun_state != communication.LIVEIMU:
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
    else:
        plt.close(fig=fig_plotLiveIMU)
        reader_thd.setContSendAndReceive()


def on_press(event):
    if event.key == 'up':
        robot.command = communication.FORWARD
    elif event.key == 'down':
        robot.command = communication.BACKWARD
    elif event.key == 'left':
        robot.command = communication.LEFT
    elif event.key == 'right':
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
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
    if reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)
    reader_thd.stop()
    print(goodbye)

def handle_close_plotDC(evt):
    if reader_thd.commun_state == communication.DCCALIBRATION:
        plt.close(fig=fig_plotDCaptor)
        reader_thd.stop_reading()

def handle_close_plotLIMU(evt):
    if reader_thd.commun_state == communication.LIVEIMU:
        plt.close(fig=fig_plotLiveIMU)
        reader_thd.setContSendAndReceive()


#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig_r.canvas.draw_idle()
        reader_thd.plot_updated()
    if reader_thd.commun_state == communication.DCCALIBRATION:
        fig_plotDCaptor.canvas.draw() 
    if reader_thd.commun_state == communication.LIVEIMU:
        fig_plotLiveIMU.canvas.draw() 
    

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
    captorDistButton        = Button(plotCaptorDistAx, 'Calib Dist Captor', color=colorAx, hovercolor='0.975')
    captorLIMUButton        = Button(plotLiveIMUAx, 'Live IMU', color=colorAx, hovercolor='0.975')
    stop                    = Button(stopAx, 'Stop', color=colorAx, hovercolor='0.975')

    ax_r.set_aspect('equal', adjustable='box')

    resetButton.on_clicked(reset)
    sendAndReceiveButton.on_clicked(sendAndReceiveCallback)
    captorDistButton.on_clicked(plotDCCallback)
    captorLIMUButton.on_clicked(plotLiveIMUCallback)
    stop.on_clicked(stop_readingCallback)

    
    sizefromrobot = 5 * robot_diameter

    ax_r.set_xlim([-sizefromrobot + robot.position.getx(), sizefromrobot + robot.position.getx()])
    ax_r.set_ylim([-sizefromrobot + robot.position.gety(), sizefromrobot + robot.position.gety()])

    plt.show()


def main():
    # #test if the serial port as been given as argument in the terminal
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

    global fig_r
    fig_r, ax_r = plt.subplots(figsize=(15.36, 7.624))

    global robot
    robot = Robot(fig_r, ax_r)
    robot.command = communication.NEUTRAL

    # #serial reader thread config
    # #begins the serial thread
    global reader_thd
    reader_thd = serial_thread(port , robot)
    reader_thd.start()

    plotMovobot(fig_r, ax_r)


if __name__ == "__main__":
    main()

