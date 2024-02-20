from networktables import NetworkTables
import math

class NoteDetector:
    def __init__ (self, config):
        #192.168.101.2
        NetworkTables.initialize(server='10.10.76.2')
        self.noteSub = NetworkTables.getTable('noteDetector')
        self.config = config

        self.noteSub.putString('testKey', 'stuff')
    
    def getXMin(self):
        return self.noteSub.getNumber('xmin', -1000)
    def getXMax(self):
        return self.noteSub.getNumber('xmax', -1000)
    def getYMin(self):
        return self.noteSub.getNumber('ymin', -1000)
    def getYMax(self):
        return self.noteSub.getNumber('ymax', -1000)
    def getTestMessage(self):
        return self.noteSub.getString('testKey', "nothing")
    def hasTarget(self):
        return bool(self.noteSub.getBoolean('hasTarget', False))

    def getTargetErrorX(self):
        self.targetPixelX = ((self.getXMin() + self.getXMax())/2 - self.config['CAMERA_PIXELS_X']/2)
        self.targetAngleX = (self.targetPixelX/(self.config['CAMERA_PIXELS_X']/2))*(self.config['CAMERA_FOV_X']/2)
        self.targetErrorX = self.getTargetErrorY() * math.tan(math.radians(self.targetAngleX))
        return self.targetErrorX

    def getTargetErrorY(self):
        self.targetPixelZ = (self.config['CAMERA_PIXELS_Z'] - (self.getYMin() + self.getYMax())/2) - self.config['CAMERA_PIXELS_Z']/2
        self.targetAngleZ = (self.targetPixelZ/(self.config['CAMERA_PIXELS_Z']/2))*(self.config['CAMERA_FOV_Z']/2) + self.config['CAMERA_ANGLE_ABOVE_HORIZONTAL']
        self.targetErrorY = ((self.config['CAMERA_HEIGHT'])/math.tan(math.radians(self.targetAngleZ)))*(-1) + self.config['CAMERA_OFFSET_Y']
        return self.targetErrorY
    
    

    