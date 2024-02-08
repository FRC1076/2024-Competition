
import sys
import wpilib
import wpimath.controller
from wpilib import interfaces
import rev
import ctre

from robotconfig import robotConfig
from controller import Controller
from mechanism import Mechanism

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        controllers = self.controllerInit(robotConfig["CONTROLLERS"])
        self.driver = controllers[0]
        self.operator = controllers[1]
        self.mechanism = Mechanism(robotConfig["MECHANISM"])
        return
    
    def controllerInit(self, config):
        ctrls = []
        self.log(config)
        for ctrlConfig in config.values():
            self.log(ctrlConfig)
            ctrlID = ctrlConfig['ID']
            ctrl = wpilib.XboxController(ctrlID)
            dz = ctrlConfig['DEADZONE']
            lta = ctrlConfig['LEFT_TRIGGER_AXIS']
            rta = ctrlConfig['RIGHT_TRIGGER_AXIS']
            ctrls.append(Controller(ctrl, dz, lta, rta))
        return ctrls
    
    def teleopInit(self):
        return

    def robotPeriodic(self):
        return True
    
    def teleopPeriodic(self):
        #intake motor
        if self.operator.xboxController.getLeftBumper():
            self.mechanism.intakeNote()
        else:
            self.mechanism.stopIntake()
            self.mechanism.stopIndexing()   
        
        #shooter motor and sprocket
        if self.operator.xboxController.getRightBumper():
            self.mechanism.shootNote()
        else:
            self.mechanism.stopShooting()
        
        #rotate sprocketDown
        if self.operator.xboxController.getLeftTriggerAxis():
            self.mechanism.sprocketDown()

        #rotate sprocket down
        if self.operator.xboxController.getRightTriggerAxis():
            self.mechanism.sprocketUp()

        if self.operator.xboxController.getLeftY():
            self.mechanism.rotateSprocket(self.operator.xboxController.getLeftY()) #assumes that the minimum and maximum values for the axis are -1 and 1

        return

    
    def autonomousInit(self): 
        return
    
    def autonomousPeriodic(self):
        return
    
    def log(self, *dataToLog):
        return

if __name__ == "__main__":
    if sys.argv[1] == 'sim':
        TEST_MODE = True
    wpilib.run(MyRobot)
