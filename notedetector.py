from networktables import NetworkTables
import logging

class NoteDetector:
    def __init__ (self, config):
        NetworkTables.initialize()
        self.noteSub = NetworkTables.getTable('noteDetector')

        self.noteSub.putString('testKey', "stuff")
        self.config = config

        logging.basicConfig(level=logging.DEBUG)
    
    def getXMin(self):
        return self.noteSub.getNumber('xmin', 0)
    def getXMax(self):
        return self.noteSub.getNumber('xmax', 0)
    def getYMin(self):
        return self.noteSub.getNumber('ymin', 0)
    def getYMax(self):
        return self.noteSub.getNumber('ymax', 0)
    def getTestMessage(self):
        return self.noteSub.getString('testKey', "nothing")
    def hasTarget(self):
        return bool(self.noteSub.getBoolean('hasTarget', False))

    def getTargetErrorX(self):
        return (self.getXMin + self.getXMax)/2
    def getTargetErrorY(self):
        return (self.getYMin + self.getYMax)/2
    
    

    