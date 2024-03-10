from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import BuiltInWidgets
from dashboard import Dashboard

from networktables import NetworkTables
from robotconfig import elasticConfig
from robotconfig import autonConfig

from wpilib import SmartDashboard

import wpilib
from wpilib.drive import DifferentialDrive
from wpilib.shuffleboard import Shuffleboard

TEST_MODE = False

class Elastic:

    def __init__(self,autonPlans):
        self.config = elasticConfig
        self.autonPlans = autonPlans
        self.autonConfig = autonConfig
        self.selectedTaskKey = autonConfig['B_THREE_NOTE_AMP_SIDE']
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

    def autonDisplay(self):
        """
        # A simple auto routine that drives forward a specified distance, and then stops.
        self.simpleAuto = DriveDistance(
            constants.kAutoDriveDistanceInches, constants.kAutoDriveSpeed, self.drive
        )

        # A complex auto routine that drives forward, drops a hatch, and then drives backward.
        self.complexAuto = ComplexAuto(self.drive, self.hatch)
        """
        #self.autonPlans is a list of strings

        AUTON_PLAN_LIST = {
            "B_THREE_NOTE_AMP_SIDE":self.autonConfig["B_THREE_NOTE_AMP_SIDE"],
        } 

        # Chooser
        self.chooser = wpilib.SendableChooser()

        self.chooser.setDefaultOption("B_THREE_NOTE_AMP_SIDE", self.autonConfig["B_THREE_NOTE_AMP_SIDE"])
        for i in range(1,int(len(self.autonPlans))):
            print("ADDING THE THIS NUMBER TO THE SENDABLE CHOOSER:",i)
            print(self.autonPlans[i])
            AUTON_PLAN_LIST[self.autonPlans[i]] = self.autonConfig[self.autonPlans[i]]
            print("THE AUTON_PLAN_LIST",AUTON_PLAN_LIST[self.autonPlans[i]])
            #u need to get the value of the auton config terms

        print("ALRIGHT NOW WE ARE MOVING ON TO THE V FOR LOOP TO PUT THIS STUFF ON ELASTIC FR!!!")

        for key, value in AUTON_PLAN_LIST.items():
            self.chooser.addOption(key, value)

        SmartDashboard.putData(self.chooser)

    def getSelectedAuton(self):
        print(self.chooser.getSelected())

        #for index, (k,v) in enumerate(AUTON_PLAN_LIST.items()):
            #print(index,":",k,v) 
        
            #print("ADDING THE THIS TO THE ELASTIC BOAAARDDD:",v)
            #print("PLEASE WORK AUTON_PLAN_LIST",str(AUTON_PLAN_LIST))
            #print(self.autonConfig[AUTON_PLAN_LIST[v]])
            #print("PLEASE WORK AUTON_PLAN_LIST",str(AUTON_PLAN_LIST[v]), self.autonConfig[AUTON_PLAN_LIST[v]])
            #self.chooser.addOption(AUTON_PLAN_LIST[v], self.autonConfig[AUTON_PLAN_LIST[v]])

        # Put the chooser on the dashboard
        #print(AUTON_PLAN_LIST)
        #print the above generated list of filtered auton plans into the combobox

    def coralCameraServer():
        from wpilib.cameraserver import CameraServer
        """
        Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
        FRC dashboard without doing any vision processing. This is the easiest way to get camera images
        to the dashboard. Just add this to the robotInit() method in your program.
        """










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