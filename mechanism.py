
import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
from wpimath.controller import ArmFeedforward
import rev
from wpilib import DoubleSolenoid
import ctre
import rev
from beambreak import BeamBreak
import math

class Mechanism:
    def __init__(self, config) -> None:
        self.config = config

        motor_type_brushless = rev.CANSparkMaxLowLevel.MotorType.kBrushless
        motor_type_brushed = rev.CANSparkMaxLowLevel.MotorType.kBrushed
        self.intakeBeamBreak = BeamBreak(config["INTAKE_BEAMBREAK_PIN"])
        self.intakeMotor = rev.CANSparkMax(config["INTAKE_MOTOR_ID"], motor_type_brushless)
        self.indexMotor = rev.CANSparkMax(config["INDEX_MOTOR_ID"], motor_type_brushless)
        self.leftShootingMotor = rev.CANSparkMax(config["SHOOTER_LEFT_MOTOR_ID"], motor_type_brushless)
        self.rightShootingMotor = rev.CANSparkMax(config["SHOOTER_RIGHT_MOTOR_ID"], motor_type_brushless)
        self.leftShootingMotor.enableVoltageCompensation(12)
        self.rightShootingMotor.enableVoltageCompensation(12)
        # self.moveHoodMotor = rev.CANSparkMax(config["HOOD_MOTOR_ID"], motor_type_brushless)
        self.sprocketLeftMotor = rev.CANSparkMax(config["SPROCKET_LEFT_MOTOR_ID"], motor_type_brushless)
        self.sprocketRightMotor = rev.CANSparkMax(config["SPROCKET_RIGHT_MOTOR_ID"], motor_type_brushless)
        self.sprocketLeftMotor.enableVoltageCompensation(12)
        self.sprocketRightMotor.enableVoltageCompensation(12)
        self.sprocketPID = PIDController(config["SPROCKET_PID_KP"], config["SPROCKET_PID_KI"], config["SPROCKET_PID_KD"])
        self.sprocketFeedforward = ArmFeedforward(config["SPROCKET_FEEDFORWARD_KS"],config["SPROCKET_FEEDFORWARD_KG"],config["SPROCKET_FEEDFORWARD_KV"],config["SPROCKET_FEEDFORWARD_KA"])
        self.sprocketAbsoluteEncoder = wpilib.DutyCycleEncoder(config["SPROCKET_ENCODER_ID"])
        self.sprocketEncoderShift = config["SPROCKET_ENCODER_SHIFT"]
        self.sprocketEncoderZero = config["SPROCKET_ENCODER_ZERO"]
        return

    #action is intake or eject, L1 is intake, B is eject
    def intakeNote(self):
        self.intakeMotor.set(self.config["INTAKE_SPEED"])
        if self.intakeBeamBreak.beamBroken():
            print("note inside intake")
        return
    
    def stopIntake(self):
        self.intakeMotor.set(0)
    
    def ejectNote(self):
        self.intakeMotor.set(-1*self.config["INTAKE_SPEED"])
        return
    
    #do the sequence that shoots the note
    #r1 shoots the note
    def shootNote(self):
        self.leftShootingMotor.set(self.config["SHOOTER_LEFT_SPEED"])
        self.rightShootingMotor.set(self.config["SHOOTER_RIGHT_SPEED"])
        return
    
    def shootReverse(self):
        self.leftShootingMotor.set(self.config["SHOOTER_LEFT_REVERSE_SPEED"])
        self.rightShootingMotor.set(self.config["SHOOTER_RIGHT_REVERSE_SPEED"])
        return
    
    #forces stop because motor doesn't always go to 0 by itself
    def stopShooting(self):
        self.leftShootingMotor.set(0)
        self.rightShootingMotor.set(0)
        return
    
    def sprocketUp(self): #moves the shooter away from the intake
        config = self.config
        self.sprocketLeftMotor.set(config["SPROCKET_MOTOR_LEFT_UP"])
        self.sprocketRightMotor.set(config["SPROCKET_MOTOR_RIGHT_UP"])
        self.sprocketLimitStop()
        return
    
    def sprocketDown(self): #moves the shooter back to the intake
        config = self.config
        self.sprocketLeftMotor.set(config["SPROCKET_MOTOR_LEFT_DOWN"])
        self.sprocketRightMotor.set(config["SPROCKET_MOTOR_RIGHT_DOWN"])
        #self.sprocketLimitStop()
        return
    
    def sprocketToPosition(self, targetPosition): #test and debug me!
        config = self.config
        self.sprocketPIDCalculation = self.sprocketPID.calculate(self.getSprocketAngle(), targetPosition)
        self.sprocketFeedforwardCalculation = self.sprocketFeedforward.calculate(math.radians(self.getSprocketAngle()), config["SPROCKET_FEEDFORWARD_VELOCITY"], config["SPROCKET_FEEDFORWARD_ACCELERATION"])
        self.sprocketMotorSpeed = self.sprocketPIDCalculation + self.sprocketFeedforwardCalculation
        self.sprocketRightMotor.set(-self.sprocketMotorSpeed)
        self.sprocketLeftMotor.set(self.sprocketMotorSpeed)
        print(self.getSprocketAngle())
        self.sprocketLimitStop()
        return
    
    def stopSprocket(self):
        config = self.config
        self.sprocketFeedforwardCalculation = self.sprocketFeedforward.calculate(math.radians(self.getSprocketAngle()), config["SPROCKET_FEEDFORWARD_VELOCITY"], config["SPROCKET_FEEDFORWARD_ACCELERATION"])
        self.sprocketRightMotor.set(-self.sprocketFeedforwardCalculation)
        self.sprocketLeftMotor.set(self.sprocketFeedforwardCalculation)
        #print(self.sprocketFeedforwardCalculation)
    
    def sprocketLimitStop(self):
        if(self.getSprocketAngle() > 90):
            self.stopSprocket()
    
    def stopIndexing(self):
        self.indexMotor.set(0)
        return
    
    def getSprocketAngle(self):
        return (self.sprocketAbsoluteEncoder.getAbsolutePosition() * 360 + self.sprocketEncoderShift) % 360 - self.sprocketEncoderZero
    
    def indexNote(self):
        self.indexMotor.set(self.config["INDEX_SPEED"])
        return
    
    def reverseIndex(self):
        self.indexMotor.set(-1 * self.config["INDEX_SPEED"])
        return
    