from networktables import NetworkTables
import math

class NoteDetector:
    def __init__ (self, config):
        #IP of the coral
        #192.168.101.2
        NetworkTables.initialize(server='10.10.76.2')
        self.noteSub = NetworkTables.getTable('noteDetector')
        self.config = config

        # self.noteSub.putString('testKey', 'stuff')
    
    # get the camera pixel coordinates of the bounding box
    def getXMin(self):
        return self.noteSub.getNumber('xmin', -1000)
    def getXMax(self):
        return self.noteSub.getNumber('xmax', -1000)
    def getYMin(self):
        return self.noteSub.getNumber('ymin', -1000)
    def getYMax(self):
        return self.noteSub.getNumber('ymax', -1000)
    def hasTarget(self):
        return bool(self.noteSub.getBoolean('hasTarget', False))

    def getTargetErrorAngle(self):
        # angle to the note, in degrees
        # positive angle is to the right
        return math.degrees(math.atan(self.getTargetErrorX/self.getTargetErrorY))

    def getTargetErrorX(self):
        # x distance to the note, in inches
        # positive error is to the right

        self.targetPixelX = ((self.getXMin() + self.getXMax())/2 - self.config['CAMERA_PIXELS_X']/2)
        self.targetAngleX = (self.targetPixelX/(self.config['CAMERA_PIXELS_X']/2))*(self.config['CAMERA_FOV_X']/2)
        self.targetErrorX = self.getTargetErrorY() * math.tan(math.radians(self.targetAngleX)) + self.config['CAMERA_OFFSET_X']
        return self.targetErrorX

    def getTargetErrorY(self):
        # y distance to the note

        self.targetPixelZ = (self.config['CAMERA_PIXELS_Z'] - (self.getYMin() + self.getYMax())/2) - self.config['CAMERA_PIXELS_Z']/2
        self.targetAngleZ = (self.targetPixelZ/(self.config['CAMERA_PIXELS_Z']/2))*(self.config['CAMERA_FOV_Z']/2) + self.config['CAMERA_ANGLE_ABOVE_HORIZONTAL']
        self.targetErrorY = ((self.config['CAMERA_HEIGHT'] - self.config['NOTE_HEIGHT'])/math.tan(math.radians(self.targetAngleZ)))*(-1) + self.config['CAMERA_OFFSET_Y'] + self.config['CAMERA_OFFSET_Y']
        return self.targetErrorY
    
    

    