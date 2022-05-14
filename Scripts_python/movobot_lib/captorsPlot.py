# EP2-MOVOBOT
# File resposible for data management of the 
#   captors intended for ploting

# Libraries import
from time import time
import movobot_lib.robotPlot as robotPlot

TIME_BACK_IMU = -50

# Distance captor calibration class
class CaptorDist():
    def __init__(self) -> None:
        self.dist = []
        self.intensity = []

    # Add values in the lists
    def addValues(self,newvalues:list):
        new_distance = robotPlot.convert_steps_to_mm(newvalues[0])
        if new_distance not in self.dist:
            self.dist.append(new_distance)
            self.intensity.append(newvalues[1])
            self.sort()
    
    # Sort the values in increasing order
    def sort(self):
        dist_and_inten = sorted(zip(self.dist, self.intensity))
        self.dist, self.intensity = list(zip(*dist_and_inten))
        self.dist = list(self.dist)
        self.intensity = list(self.intensity) 

    # Return the lists
    def get_values(self):
        return self.dist, self.intensity

# LiveIMU Plot class
class LiveIMU():
    def __init__(self) -> None:
        self.time_l = []
        self.intensity = []
        self.previous_t = 0
        self.current_t = 0

    # Add values in the lists
    def addValue(self,newvalue):
        if self.time_l == [] or self.time_l[0] > TIME_BACK_IMU: # Beginning of the plot
            self.intensity.append(newvalue)
            self.adjust_time()
        else: # Move the plot as new values arrive
            self.intensity.append(newvalue)
            self.adjust_time()
            
            for time_, intensity in zip(self.time_l, self.intensity):
                if not time_ < TIME_BACK_IMU:
                    break
                self.time_l.remove(time_)
                self.intensity.remove(intensity)

    
    def adjust_time(self):
        dt = self.get_dt()

        for i in range(len(self.time_l)):
            self.time_l[i] -=  dt
        self.time_l.append(0)
        

    def get_dt(self):
        self.current_t = time()
        dt = self.current_t - self.previous_t
        self.previous_t = self.current_t

        return dt

    # Return the lists
    def get_values(self):
        return self.time_l, self.intensity


