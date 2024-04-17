from networktables import NetworkTables
import math

"""To Do:
Make sure noteDrive_r PID is tuned
"""

class NoteDetector:
    def __init__ (self, config, swervometer):
        #IP of the coral
        #10.10.76.16
        self.config = config
        self.swervometer = swervometer
        self.limelight = NetworkTables.getTable('limelight-note')
        self.lastHeartbeat = 0
        self.sameCounter = 0

    def hasTarget(self):
        if self.trustLimelight():
            return bool(self.limelight.getNumber('tv', 0))
        else:
            return False

    def getTargetAngleX(self):
        return self.limelight.getNumber('tx', self.config['DEFAULT_BBOX'])

    def getTargetAngleZ(self):
        return self.limelight.getNumber('ty', self.config['DEFAULT_BBOX']) + self.config['CAMERA_ANGLE_ABOVE_HORIZONTAL']

    def getHeartbeat(self):
        return self.limelight.getNumber('hb', 0)

    def trustLimelight(self):
        if self.lastHeartbeat == self.getHeartbeat():
            self.sameCounter += 1
        else:
            self.sameCounter = 0

        self.lastHeartbeat = self.getHeartbeat()

        return self.sameCounter < 11
    #4,8,
    def getTargetErrorX(self, limelight_latency=8):
        return self.getTargetErrorY(limelight_latency) * math.tan(math.radians(self.getTargetAngleX())) + self.config['CAMERA_OFFSET_X'] - self.swervometer.getDistanceTraveledX(limelight_latency + self.sameCounter)

    def getTargetErrorY(self, limelight_latency=8):
        return ((self.config['CAMERA_HEIGHT'] - self.config['NOTE_HEIGHT'])/math.tan(math.radians(self.getTargetAngleZ())))*(-1) + self.config['CAMERA_OFFSET_Y'] - self.swervometer.getDistanceTraveledY(limelight_latency + self.sameCounter)

    def getTargetErrorAngle(self):
        # angle to the note, in degrees
        # positive angle is to the right
        return math.degrees(math.atan(self.getTargetErrorX()/self.getTargetErrorY()))

    