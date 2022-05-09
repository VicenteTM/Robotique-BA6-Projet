# EP2-MOVOBOT
# File resposible for data management of the 
#   captors intended for ploting

# Libraries import
import movobot_lib.robotPlot as robotPlot

# Distance captor calibration class
class CaptorDist():
    def __init__(self) -> None:
        self.dist = []
        self.intensity = []

    # Add values in the lists
    def addValues(self,newvalues:list):
        if newvalues[0] not in self.dist:
            self.dist.append(robotPlot.convert_steps_to_mm(newvalues[0]))
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
        self.time_l = list(range(0, 51))
        self.intensity = []

    # Add values in the lists
    def addValue(self,newvalue):
        if len(self.intensity) < 51: # Beginning of the plot
            self.intensity.append(newvalue)
        else: # Move the plot as new values arrive
            self.intensity = self.intensity[1:51]
            self.intensity.append(newvalue)
    
    # Return the lists
    def get_values(self):
        return self.time_l[0:len(self.intensity)], self.intensity


