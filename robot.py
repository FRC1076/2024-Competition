
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
            self.mechanism.intakeNote()
        
        #shooter motor and sprocket
        elif self.operator.xboxController.getRightBumper():
            self.mechanism.launchNote()
        
        #rotate sprocketDown
        elif self.operator.xboxController.getLeftTriggerAxis():
            self.mechanism.sprocketDown()

        #rotate sprocket down
        elif self.operator.xboxController.getRightTriggerAxis():
            self.mechanism.sprocketUp()
        
        elif self.operator.xboxController.getBButton():
            self.mechanism.eject()

        elif self.operator.xboxController.getYButton():
            self.mechanism.launchReverse()

        else:
            self.mechanism.stopShooting()   
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
