import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
import rev
from wpilib import DoubleSolenoid
import ctre
import rev

class Mechanism:
    def __init__(self, config) -> None:
        self.config = config
        motor_type_brushless = rev.CANSparkMaxLowLevel.MotorType.kBrushless

        #motors in the shooter
        self.leftShootingMotor = rev.CANSparkMax(99,motor_type_brushless) #fix the device id later
        self.rightShootingMotor = rev.CANSparkMax(99,motor_type_brushless)

        #intake motor (pulls the notes in)
        self.intakeMotor = rev.CANSparkMax(99,motor_type_brushless)

        #intake up or down motor
        self.intakeUpDownMotor = rev.CANSparkMax(99,motor_type_brushless)

        #motor that moves the hood
        self.moveHoodMotor = rev.CANSparkMax(99,motor_type_brushless)
        return
    
    def moveHood(self,position):
        #move the hood (part that allows scoring in the amp)
        #position is forward or back
        
        #b is hood back, x is hood forward
        return

    def shootNote(self):
        #do the sequence that shoots the note

        #a shoots the note

        return

    def intakeNote(self,action):
        #intake a note
        #action is intake or eject

        #L1 is intake, R1 is eject
        return
    
    def intakeUpDown(self,position):
        #move the intake piece up or down

        #D-pad up for up, D-pad down for down
        return

    def transportNote(self,direction):
        #direction is up or down

        #L2 is up, R2 is down
        return