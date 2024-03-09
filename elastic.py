from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import BuiltInWidgets
from dashboard import Dashboard

from networktables import NetworkTables
from robotconfig import elasticConfig

TEST_MODE = False

class Elastic:

    def __init__(self):
        self.config = elasticConfig
        self.isNoteDetected = self.config["NOTE_IS_DETECTED"]
        self.isNoteLeft = self.config["NOTE_ON_LEFT"]
        self.isNoteRight = self.config["NOTE_ON_RIGHT"]
        self.isNoteLoaded = self.config["NOTE_IS_LOADED"]

        #self.elasticboard = Dashboard.getDashboard(testMode=TEST_MODE)
        #self.elasticboard = NetworkTables.getTable('SmartDashboard')

    def displayMainWindow(self):
        tab = "Main Window"

        (Shuffleboard.getTab(tab)
        .add("Note Detected!", self.isNoteDetected)
        .withSize(4,4)
        .withPosition(17,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

        (Shuffleboard.getTab(tab)
        .add("Note on Left", self.isNoteLeft)
        .withSize(4,4)
        .withPosition(13,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

        (Shuffleboard.getTab(tab)
        .add("Note on Right", self.isNoteRight)
        .withSize(4,4)
        .withPosition(21,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

        (Shuffleboard.getTab(tab)
        .add("Note Loaded!",self.isNoteLoaded)
        .withSize(4,4)
        .withPosition(17,4)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())
    
    def summonTheButtons(self):
        tab = "Main Window"

        self.buttonA = (Shuffleboard.getTab(tab)
        .add("Note Detected!",False)
        .withSize(4,3)
        .withPosition(0,8)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry())

        self.buttonB = (Shuffleboard.getTab(tab)
        .add("Note on Right",False)
        .withSize(4,3)
        .withPosition(4,8)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry())

        self.buttonC = (Shuffleboard.getTab(tab)
        .add("Note on Left",False)
        .withSize(4,3)
        .withPosition(8,8)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry())

        self.buttonD = (Shuffleboard.getTab(tab)
        .add("Note Loaded!",False)
        .withSize(4,3)
        .withPosition(12,8)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry())

"""
    def updateCoral():
        self.isNoteDetected = 
        self.isNoteLeft = self.config["NOTE_ON_LEFT"]
        self.isNoteRight = self.config["NOTE_ON_RIGHT"]
        self.isNoteLoaded = self.config["NOTE_IS_LOADED"]
        return
    
    def updateLimelight():
        return

"""

"""
    def elasticTesting(self):
        #how can I get if the buttons are true or not and add that boolean to a variable
        self.isNoteLeft = self.buttonA.getBoolean()
        self.isNoteRight = True
        self.isNoteDetected = True
        self.isNoteLoaded = True

"""
    
    #self.isNoteDetected = self.elasticboard.getBoolean("NOTE_DETECTED",False)
"""
        self.isNoteDetected = (Shuffleboard.getTab("Main Window")
        .addBoolean("NOTE_IS_DETECTED",False)
        .withWidget(BuiltInWidgets.kToggleButton)
        .getEntry())

        self.isNoteLeft = self.buttonB
        self.isNoteRight = self.buttonC
        self.isNoteLoaded = self.buttonD
        print(self.config["NOTE_IS_DETECTED"],self.config["NOTE_ON_LEFT"],
              self.config["NOTE_ON_RIGHT"],self.config["NOTE_IS_LOADED"])
        print(self.isNoteDetected,self.isNoteLeft,self.isNoteRight,self.isNoteLoaded)
    
        """

    #def updateDisplay(self):
        #(Shuffleboard.getTab(tab).

"""
        self.isNoteDetected = self.buttonA.getBoolean("Main Window","NOTE_IS_DETECTED")
        self.isNoteLeft = self.buttonB.getBoolean("Main Window","NOTE_ON_LEFT")
        self.isNoteRight = self.buttonC.getBoolean("Main Window","NOTE_ON_RIGHT")
        self.isNoteLoaded = self.buttonD.getBoolean("Main Window","NOTE_IS_LOADED")
        """
        
    
    #self.dashboard.getBoolean(moduleName + '/' + key, defaultValue)
    #self.dashboard.getBoolean(DASH_PREFIX,'Team is Red'):
   # def noteAlignment():
        #if camera detects note on left
        #(Shuffleboard.("Main Window","Note"))
        

        #if camera detects note on right

"""
.add("Note Loaded!", 1)
        .withSize(4,4)
        .withPosition(21,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

        (Shuffleboard.getTab(tab)
        .add("Limelight", 1) 
        .withSize(10,10)
        .withPosition(0,0)
        .withWidget(widgetType=str)
        .getEntry())
"""