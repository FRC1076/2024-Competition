from wpilib import SmartDashboard
import wpilib

class Elastic:

    def __init__(self, autonPlans, activeStartingPositions):
        self.autonPlans = autonPlans
        self.startingPositions = activeStartingPositions
        self.isElasticSubmitted = False
        self.beamBreakBoolean = False
    
    def teamDisplay(self, defaultColor, defaultLabel):
        self.teamChooser = wpilib.SendableChooser()
        color = (defaultColor + " Alliance")
        label = defaultLabel
        self.teamChooser.setDefaultOption(color, label)
        if defaultColor == 'Red':
            self.teamChooser.addOption("Blue Alliance", False)
        else:
            self.teamChooser.addOption("Red Alliance", True)
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
        self.putBoolean("NOTE LOADED", noteDetected)

    def updateControllerConnectedDisplay(self, driverConnected, operatorConnected):
        self.putBoolean("Driver Connected", driverConnected)
        self.putBoolean("Operator Connected", operatorConnected)

    def updateHasTargetDisplay(self, hasTarget):
        self.putBoolean("hasTarget", hasTarget)

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