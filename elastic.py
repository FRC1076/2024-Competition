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
        self.selectedTaskKey = autonConfig['B_FAST_FOUR_NOTE_SUBWOOFER']
        self.isNoteDetected = self.config["NOTE_IS_DETECTED"]
        self.isNoteLeft = self.config["NOTE_ON_LEFT"]
        self.isNoteRight = self.config["NOTE_ON_RIGHT"]
        self.isNoteLoaded = self.config["NOTE_IS_LOADED"]

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

    def autonDisplay(self, default):
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption(default, default)

        for plan in self.autonPlans:
            self.chooser.addOption(plan, plan)

        SmartDashboard.putData(self.chooser)

    def getSelectedAuton(self):
        return self.chooser.getSelected()
    
    def putField(self, field):
        SmartDashboard.putData(field)

    def putNumber(self, key, num):
        SmartDashboard.putNumber(key, num)