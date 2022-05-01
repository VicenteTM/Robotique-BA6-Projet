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
            self.dist.append(newvalues[0])
            self.intensity.append(newvalues[1])
            self.sort()
    
    def get_values(self):
        return self.dist, self.intensity
