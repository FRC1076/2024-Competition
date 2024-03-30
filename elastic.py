from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import BuiltInWidgets

from robotconfig import elasticConfig
from robotconfig import autonConfig

from wpilib import SmartDashboard

import wpilib
from wpilib.shuffleboard import Shuffleboard

class Elastic:

    def __init__(self, autonPlans, activeStartingPositions):
        self.config = elasticConfig
        self.autonPlans = autonPlans
        self.startingPositions = activeStartingPositions
        self.autonConfig = autonConfig
        self.selectedTaskKey = autonConfig['B_THREE_NOTE_AMP_SIDE']
        self.isElasticSubmitted = False
        self.controllerDriverElastic = False
        self.controllerOperatorElastic = False
        self.beamBreakBoolean = False

    def displayMainWindow(self):
        """
        tab = "Main Window"
        (Shuffleboard.getTab(tab)
        .add("Driver Controller", self.controllerDriverElastic)
        .withSize(2,2)
        .withPosition(0,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())
        """
        return
    
    def teamDisplay(self, defaultColor, defaultLabel):
        self.teamChooser = wpilib.SendableChooser()
        color = (defaultColor + " Alliance")
        label = defaultLabel
        self.teamChooser.setDefaultOption(color, label)
        if defaultColor == 'Red':
            self.teamChooser.addOption("Blue Alliance", "BLUE")
        else:
            self.teamChooser.addOption("Red Alliance", "RED")
        SmartDashboard.putData(self.teamChooser)
    
    def positionDisplay(self, defaultPosition):
        self.positionChooser = wpilib.SendableChooser() 
        position = ("Positon " + defaultPosition)
        defaultLabel = defaultPosition
        self.positionChooser.setDefaultOption(position, defaultLabel)
        for i in self.startingPositions:
            position = ("Positon " + i)
            label = i
            if not(defaultLabel == label):
                self.positionChooser.addOption(position, label)
        SmartDashboard.putData(self.positionChooser)

    def autonDisplay(self):
        self.chooser = wpilib.SendableChooser()
        self.chooser.setDefaultOption(self.autonPlans[0], self.autonPlans[0])
        for plan in self.autonPlans:
            self.chooser.addOption(plan, plan)
        SmartDashboard.putData(self.chooser)
    
    def updateBeamDisplay(self, noteDetected):
        if noteDetected == True:
            self.beamBreakBoolean = True
            print("beambreak true")
        elif noteDetected == False:
            self.beamBreakBoolean = False
            print("beambreak false")
        else:
            print("beambreak NOT WORKING")
            
        self.putBoolean("NOTE LOADED", self.beamBreakBoolean) #this works

    def updateControllerConnectedDisplay(self, driverConnected, operatorConnected):
        if driverConnected:
            self.controllerDriverElastic = True
            SmartDashboard.putBoolean("Driver Connected", self.controllerDriverElastic)
        else:
            self.controllerDriverElastic = False
            SmartDashboard.putBoolean("Driver Connected", self.controllerDriverElastic)
        if operatorConnected:
            self.controllerOperatorElastic = True
            SmartDashboard.putBoolean("Operator Connected", self.controllerOperatorElastic)
        else:
            self.controllerOperatorElastic = False
            SmartDashboard.putBoolean("Operator Connected", self.controllerOperatorElastic)

    def getSelectedAuton(self):
        return self.chooser.getSelected()
    
    def getSelectedTeam(self):
        return self.teamChooser.getSelected()
    
    def getSelectedPosition(self):
        return self.positionChooser.getSelected()
   
    def putField(self, field):
        SmartDashboard.putData(field)

    def putNumber(self, key, num):
        SmartDashboard.putNumber(key, num)

    def putBoolean(self, key, bool):
        SmartDashboard.putBoolean(key, bool)