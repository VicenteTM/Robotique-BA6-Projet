import time
import matplotlib.pyplot as plt
import numpy as np
from threading import Thread


def newplot(init=True):
    # global newFigure
    # newFigure = plt.figure(300)
    # global plor
    # ax = newFigure.add_subplot(111)
    # plor, = ax.plot([], [], 'ro')
    # plt.axis([0, 1, 0, 1])
    # plt.show()
    if init:
        newplot.x = np.linspace(0, 6*np.pi, 100)
        newplot.y = np.sin(newplot.x)

        plot.show()

        newplot.fig = plt.figure()
        newplot.ax = newplot.fig.add_subplot(111)
        newplot.line1, = newplot.ax.plot(newplot.x, newplot.y, 'r-') # Returns a tuple of line objects, thus the comma
        for phase in np.linspace(0, 10*np.pi, 500):
            newplot.line1.set_ydata(np.sin(newplot.x + phase))
            newplot.fig.canvas.draw()
        newplot.fig.canvas.flush_events()

def plot(dist, intensity):
    # print(dist)
    # print(intensity)
    # plor.set_ydata(np.array(intensity))
    # plor.set_xdata(np.array(dist))
    # newFigure.canvas.draw()
    # newFigure.canvas.flush_events()
    newplot.line1.set_ydata(np.sin(intensity + dist))
    newplot.fig1.canvas.draw()
    newplot.fig1.canvas.flush_events()
    plt.show()
    return

class plotDistCaptor(Thread):

    #init function called when the thread begins
    def __init__(self):
        Thread.__init__(self)
        self.dist = []
        self.intensity = []
        self.alive = True

    #function called after the init
    def run(self):
        x = np.linspace(0, 10*np.pi, 100)
        y = np.sin(x)
        plt.ion()
        fig = plt.figure()
        ax = fig.add_subplot(111)
        line1, = ax.plot(x, y, 'ro')
        while(self.alive):
            for phase in np.linspace(0, 10*np.pi, 100):
                line1.set_ydata(self.dist)
                fig.canvas.draw()
                fig.canvas.flush_events()
            time.sleep(1)



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