
import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
from wpimath.controller import ArmFeedforward
import rev
from wpilib import DoubleSolenoid
import rev
from beambreak import BeamBreak
import math

class Mechanism:
    def __init__(self, config) -> None:
        self.config = config

        motor_type_brushless = rev.CANSparkLowLevel.MotorType.kBrushless
        motor_type_brushed = rev.CANSparkLowLevel.MotorType.kBrushed
        self.intakeMotor = rev.CANSparkMax(config["INTAKE_MOTOR_ID"], motor_type_brushless)
        self.indexMotor = rev.CANSparkMax(config["INDEX_MOTOR_ID"], motor_type_brushless)
        self.indexEncoder = self.indexMotor.getEncoder()
        self.inARollBack = False
        self.rollBackStartValue = 0
        self.leftShootingMotor = rev.CANSparkMax(config["SHOOTER_LEFT_MOTOR_ID"], motor_type_brushless)
        self.rightShootingMotor = rev.CANSparkMax(config["SHOOTER_RIGHT_MOTOR_ID"], motor_type_brushless)
        self.leftShootingEncoder = self.leftShootingMotor.getEncoder()
        self.rightShootingEncoder = self.rightShootingMotor.getEncoder()
        self.leftShootingMotor.enableVoltageCompensation(12)
        self.rightShootingMotor.enableVoltageCompensation(12)
        self.leftShootingMotor.setOpenLoopRampRate(config["SHOOTER_OPEN_LOOP_RAMP_RATE"])
        self.rightShootingMotor.setOpenLoopRampRate(config["SHOOTER_OPEN_LOOP_RAMP_RATE"])
        self.leftShooterPID = PIDController(0.0002, 0, 0)
        self.rightShooterPID = PIDController(0.0003, 0.0001, 0.0001) #0.0007, 0, 0.0005
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
        self.indexingBeam = BeamBreak(config["INTAKE_BEAMBREAK_PIN"])
        self.autonSprocketPosition = -38
        self.shooting = False
        self.climbMotor = rev.CANSparkMax(config["CLIMB_MOTOR_ID"], motor_type_brushless)
        return

    #action is intake or eject, L1 is intake, B is eject
    def intakeNote(self):
        self.intakeMotor.set(self.config["INTAKE_SPEED"])
        return
    
    def reverseIntake(self):
        self.intakeMotor.set(-self.config["INTAKE_SPEED"])
        return
    
    def stopIntake(self):
        self.intakeMotor.set(0)
    
    def ejectNote(self):
        self.intakeMotor.set(-1*self.config["INTAKE_SPEED"])
        return
    
    #do the sequence that shoots the note
    #r1 shoots the note
    def shootNote(self):
        #self.leftShootingMotor.set(self.config["SHOOTER_LEFT_SPEED"])
        self.setLeftShooterRPM(-4000)
        self.setRightShooterRPM(5000)
        #self.rightShootingMotor.set(self.config["SHOOTER_RIGHT_SPEED"])
        return
    
    def shootAmp(self):
        #self.leftShootingMotor.set(self.config["SHOOTER_LEFT_SPEED"])
        self.setLeftShooterRPM(-2000)
        self.setRightShooterRPM(2000)
        #self.rightShootingMotor.set(self.config["SHOOTER_RIGHT_SPEED"])
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

    def sprocketFullSpeedDown(self):
        self.sprocketLeftMotor.set(-1)
        self.sprocketRightMotor.set(1)
    
    def sprocketToPosition(self, targetPosition): #test and debug me!
        config = self.config
        self.sprocketPIDCalculation = self.sprocketPID.calculate(self.getSprocketAngle(), targetPosition)
        self.sprocketFeedforwardCalculation = self.sprocketFeedforward.calculate(math.radians(self.getSprocketAngle()), config["SPROCKET_FEEDFORWARD_VELOCITY"], config["SPROCKET_FEEDFORWARD_ACCELERATION"])
        if(targetPosition <= -30):
            self.sprocketPIDCalculation /= 2
        self.sprocketMotorSpeed = self.sprocketPIDCalculation + self.sprocketFeedforwardCalculation
        self.sprocketRightMotor.set(-self.sprocketMotorSpeed)
        self.sprocketLeftMotor.set(self.sprocketMotorSpeed)
        #print(self.getSprocketAngle())
        self.sprocketLimitStop()
        return abs(targetPosition - self.getSprocketAngle()) < 0.5
    
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
        self.inARollBack = False
        self.indexMotor.set(self.config["INDEX_SPEED"])
        return
    
    def reverseIndex(self):
        self.inARollBack = False
        self.indexMotor.set(-1 * self.config["INDEX_SPEED"])
        return
    
    def indexFixedRollBack(self):
        if not self.inARollBack:
            self.inARollBack = True
            self.rollBackStartValue = self.indexEncoder.getPosition()

    def indexBeamBroken(self):
        return self.indexingBeam.beamBroken()

    def indexBeamHealthy(self):
        return self.indexingBeam.isSelfCheckHealthy()


    def periodic(self):
        if self.inARollBack:
            self.indexMotor.set(-self.config["INDEX_SPEED"])
            if(self.indexEncoder.getPosition() < self.rollBackStartValue - self.config["INDEX_ROLL_BACK_ROTATIONS"]):
                self.inARollBack = False
                self.indexMotor.set(0)

    def getShooterRPM(self):
        return self.leftShootingEncoder.getVelocity(), self.rightShootingEncoder.getVelocity()

    def setLeftShooterRPM(self, rpm):
        self.leftShootingMotor.set(rpm / 5100 + self.leftShooterPID.calculate(self.leftShootingEncoder.getVelocity(), rpm))

    def setRightShooterRPM(self, rpm):
        self.rightShootingMotor.set(rpm / 5100 + self.rightShooterPID.calculate(self.rightShootingEncoder.getVelocity(), rpm))
        #print(self.rightShooterPID.calculate(self.rightShootingEncoder.getVelocity(), rpm))

    def setAutonSprocketPosition(self, position):
        self.autonSprocketPosition = position

    def setShootState(self, isShooting):
        self.shooting = isShooting

    def autonPeriodic(self):
        self.sprocketToPosition(self.autonSprocketPosition)
        if self.indexBeamBroken() and not self.shooting:
            self.stopIndexing()
        else:
            self.indexNote()
    
    def lockClimb(self):
        self.climbMotor.set(-0.2)
    
    def stopClimb(self):
        self.climbMotor.set(0)

    def reverseClimb(self):
        self.climbMotor.set(0.2)