
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
        self.indexMotor = rev.CANSparkMax(config["INDEX_MOTOR_ID"], motor_type_brushless)
        self.leftShootingMotor = rev.CANSparkMax(config["SHOOTER_LEFT_MOTOR_ID"], motor_type_brushless)
        self.rightShootingMotor = rev.CANSparkMax(config["SHOOTER_RIGHT_MOTOR_ID"], motor_type_brushless)
        self.leftShootingMotor.enableVoltageCompensation(12)
        self.rightShootingMotor.enableVoltageCompensation(12)
        # self.moveHoodMotor = rev.CANSparkMax(config["HOOD_MOTOR_ID"], motor_type_brushless)
        self.sprocketMotor = rev.CANSparkMax(config["SPROCKET_MOTOR_ID"], motor_type_brushless)
        self.sprocketPID = PIDController(config["SPROCKET_PID_KP"], config["SPROCKET_PID_KI"], config["SPROCKET_PID_KD"])
        self.sprocketAbsoluteEncoder = wpilib.DutyCycleEncoder(config["SPROCKET_ENCODER_ID"])
        self.sprocketEncoderZero = config["SPROCKET_ENCODER_ZERO"]
        return

    #action is intake or eject, L1 is intake, B is eject
    def intakeNote(self):
        self.intakeMotor.set(self.config["INTAKE_SPEED"])
        if self.intakeBeamBreak.beamBroken():
            print("note inside intake")
        self.indexMotor.set(self.config["INDEX_SPEED"])
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
        self.leftShootingMotor.set(-1*self.config["SHOOTER_LEFT_SPEED"])
        self.rightShootingMotor.set(-1*self.config["SHOOTER_RIGHT_SPEED"])
        return
    
    #forces stop because motor doesn't always go to 0 by itself
    def stopShooting(self):
        self.leftShootingMotor.set(0)
        self.rightShootingMotor.set(0)
        return
    
    def sprocketUp(self): #moves the shooter away from the intake
        self.sprocketMotor.set(self.config["SPROCKET_MOTOR_UP"])
        return
    
    def sprocketDown(self): #moves the shooter back to the intake
        self.sprocketMotor.set(self.config["SPROCKET_MOTOR_DOWN"])
        return
    
    def rotateSprocket(self, speed):
        """
        if (getSprocketAngle() > 0 and getSprocketAngle < 150):
            self.sprocketMotor.set(speed)


        #this is in a comment because the angles in the if statement need to be adjusted in testing
        """
        self.sprocketMotor.set(speed)
        return

    def sprocketToPosition(self, targetPosition):
        self.sprocketMotorSpeed = self.sprocketPID.calculate(self.getSprocketAngle(), targetPosition)
        self.sprocketMotor.set(self.sprocketMotorSpeed)
        return
    
    def stopIndexing(self):
        self.indexMotor.set(0)
        return
    
    def getSprocketAngle(self):
        return self.sprocketAbsoluteEncoder.getAbsolutePosition() * 360 - self.sprocketEncoderZero