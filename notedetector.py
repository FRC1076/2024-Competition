from networktables import NetworkTables

class NoteDetector:
    def __init__ (self):
        NetworkTables.initialize()
        self.noteSub = NetworkTables.getTable('noteDetector')
    
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
    

    