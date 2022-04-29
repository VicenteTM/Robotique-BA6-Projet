import matplotlib.pyplot as plt
import numpy as np


def newplot():
    global newFigure
    newFigure = plt.figure(300)
    global plor
    ax = newFigure.add_subplot(111)
    plor, = ax.plot([], [], 'ro')
    plt.axis([0, 1, 0, 1])
    plt.show()

def plot(dist, intensity):
    print(dist)
    print(intensity)
    plor.set_ydata(np.array(intensity))
    plor.set_xdata(np.array(dist))
    newFigure.canvas.draw()
    newFigure.canvas.flush_events()
    return