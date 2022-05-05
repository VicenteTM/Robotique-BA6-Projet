import robotPlot

class CaptorDist():
    def __init__(self) -> None:
        self.dist = []
        self.intensity = []
    
    def sort(self):
        dist_and_inten = sorted(zip(self.dist, self.intensity))
        self.dist, self.intensity = list(zip(*dist_and_inten))
        self.dist = list(self.dist)
        self.intensity = list(self.intensity) 

    def addValues(self,newvalues:list):
        if newvalues[0] not in self.dist:
            self.dist.append(robotPlot.convert_steps_to_mm(newvalues[0]))
            self.intensity.append(newvalues[1])
            self.sort()
    
    def get_values(self):
        return self.dist, self.intensity

class LiveIMU():
    def __init__(self) -> None:
        self.time_l = list(range(0, 51))
        self.intensity = []

    def addValue(self,newvalue):
        if len(self.intensity) < 51:
            self.intensity.append(newvalue)
        else:
            self.intensity = self.intensity[1:51]
            self.intensity.append(newvalue)
    
    def get_values(self):
        return self.time_l[0:len(self.intensity)], self.intensity


