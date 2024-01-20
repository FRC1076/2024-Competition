import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
import rev
from wpilib import DoubleSolenoid
import ctre
import rev
from beambreak import BeamBreak

class Mechanism:
    def __init__(self, config) -> None:
        self.config = config

        motor_type_brushless = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        motor_type_brushed = rev.CANSparkMaxLowLevel.MotorType.kBrushed
        self.intakeBeamBreak = BeamBreak(config["INTAKE_BEAMBREAK_PIN"])
        self.intakeMotor = rev.CANSparkMax(config["INTAKE_MOTOR_ID"], motor_type_brushless)
        self.transportMotor = rev.CANSparkMax(config["TRANSPORT_MOTOR_ID"], motor_type_brushed)
        self.leftShootingMotor = rev.CANSparkMax(config["SHOOTER_LEFT_MOTOR_ID"], motor_type_brushless)
        self.rightShootingMotor = rev.CANSparkMax(config["SHOOTER_RIGHT_MOTOR_ID"], motor_type_brushless)
        self.moveHoodMotor = rev.CANSparkMax(config["HOOD_MOTOR_ID"], motor_type_brushless)
        return

    #action is intake or eject, L1 is intake, R1 is eject
    def intakeNote(self,action):
        self.intakeMotor.set(self.config["INTAKE_SPEED"])
        if self.intakeBeamBreak.beamBroken():
            print("note inside intake")
        return

    #moves note across the indexer
    def transportNote(self,direction):
        self.transportMotor.set(self.config["TRANSPORT_SPEED"])
        return
    
    #do the sequence that shoots the note
    #a shoots the note
    def shootNote(self):
        self.leftShootingMotor.set(self.config["SHOOTER_LEFT_SPEED"])
        self.rightShootingMotor.set(self.config["SHOOTER_RIGHT_SPEED"])
        return
    
    #move the hood (part that allows scoring in the amp)
    #position is forward or back
    #b is hood back, x is hood forward
    def moveHood(self,position):
        self.moveHoodMotor.set(self.config["HOOD_SPEED"])
        return
    
    #forces stop because motor doesn't always go to 0 by itself
    def stopShooting(self):
        self.leftShootingMotor.set(0)
        self.rightShootingMotor.set(0)
        return