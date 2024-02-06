from networktables import Networktables

class NoteDetector:
    def __init__ (self):
        Networktables.initialize()
        self.noteSub = Networktables.getTable('noteDetector')
    
    def getXMin():
        return self.noteSub.getNumber('xmin', 0)
    def getXMax():
        return self.noteSub.getNumber('xmax', 0)
    def getYMin():
        return self.noteSub.getNumber('ymin', 0)
    def getYMax():
        return self.noteSub.getNumber('ymax', 0)
    def getTestMessage():
        return self.noteSub.getString('testKey')
    

    