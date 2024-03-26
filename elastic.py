from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import BuiltInWidgets

from robotconfig import elasticConfig
from robotconfig import autonConfig

from wpilib import SmartDashboard

import wpilib
from wpilib.shuffleboard import Shuffleboard

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
        self.isElasticSubmitted = False
        self.submitButton = False
        self.testButton = False

    def displayMainWindow(self):
        tab = "Main Window"
        (Shuffleboard.getTab(tab)
        .add("Submit",self.submitButton)
        .withSize(4,4)
        .withPosition(0,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

        (Shuffleboard.getTab(tab)
        .add("Test",self.testButton)
        .withSize(4,4)
        .withPosition(0,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

    def teamDisplay(self):
        self.teamChooser = wpilib.SendableChooser()
        self.teamChooser.setDefaultOption("Green", 0)
        self.teamChooser.addOption("Red", True)
        self.teamChooser.addOption("Blue", False)
        SmartDashboard.putData(self.teamChooser)

    def autonDisplay(self):
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption(self.autonPlans[0], self.autonPlans[0])

        for plan in self.autonPlans:
            self.chooser.addOption(plan, plan)

        SmartDashboard.putData(self.chooser)

    def getSelectedAuton(self):
        return self.chooser.getSelected()
    
    def printThis(self):
        x = SmartDashboard.getBoolean("submitButton", False)
        if x==True:
            return "TRUE"
        elif x==False:
            return "FALSE"
        else:
            return "NOTHING"


    def elasticSubmitCheck(self):
        if SmartDashboard.getBoolean("Main Window",self.submitButton):
            return "True"
        
        elif SmartDashboard.getBoolean("Main Window",self.submitButton) == False:
            return "False"
        
        else:
            return "NOTHING"