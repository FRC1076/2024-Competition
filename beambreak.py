import wpilib 

class BeamBreak:
    def __init__(self, inputPin):
        self.sensor = wpilib.DigitalInput(inputPin)

    def isSelfCheckHealthy(self):
        return self.sensor.get()
    
    def beamBroken(self):
        return not(self.sensor.get())