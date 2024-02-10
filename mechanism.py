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
        self.leftShootingMotor = rev.CANSparkMax(config["LEFT_SHOOTING_MOTOR_ID"], motor_type_brushless) #fix the device id later
        self.rightShootingMotor = rev.CANSparkMax(config["RIGHT_SHOOTING_MOTOR_ID"], motor_type_brushless)
        self.indexMotor = rev.CANSparkMax(config["INDEX_MOTOR_ID"], motor_type_brushless)

        self.leftShootingEncoder = self.leftShootingMotor.getEncoder()
        self.rightShootingEncoder = self.rightShootingMotor.getEncoder()
        #intake motor (pulls the notes in)
        #self.intakeMotor = rev.CANSparkMax(3, motor_type_brushless)

        #intake up or down motor
        #self.intakeUpDownMotor = rev.CANSparkMax(4, motor_type_brushless)

        #motor that moves the hood
        #self.moveHoodMotor = rev.CANSparkMax(5, motor_type_brushless)
        return
    
    def moveHood(self,position):
        #move the hood (part that allows scoring in the amp)
        #position is forward or back
        
        #b is hood back, x is hood forward
        return

    def shootNote(self):
        #do the sequence that shoots the note

        #a shoots the note
        self.leftShootingMotor.set(self.config["LEFT_EJECT_SPEED"])
        self.rightShootingMotor.set(self.config["RIGHT_EJECT_SPEED"])
        return
    
    def stopShooting(self):
        self.leftShootingMotor.set(0)
        self.rightShootingMotor.set(0)
        return

    def shootingMotorRPMs(self):
        return self.leftShootingEncoder.getVelocity(), self.rightShootingEncoder.getVelocity()

    def intakeNote(self,action):
        #intake a note
        #action is intake or eject
        print("hola amogis :)")

        #L1 is intake, R1 is eject
        return
    
    def intakeUpDown(self,position):
        #move the intake piece up or down

        #D-pad up for up, D-pad down for down
        return

    def transportNote(self,direction):
        self.direction = direction
        #direction is up or down
        if self.direction == "up":
            self.indexMotor.set(self.config["INDEX_MOTOR_UP_SPEED"])
            print("transporting up")
        elif self.direction =="down":
            self.indexMotor.set(self.config["INDEX_MOTOR_DOWN_SPEED"])
            print("transporting down")
        else:
            print("invalid index direction")

        #L2 is up, R2 is down
        return
    
    def stopTransporting(self):
        self.indexMotor.set(0)
    
    def getShooterMotorCurrentsAsGraph(self):
        return ("{}".format(
            "*"*(int(10*self.leftShootingMotor.getOutputCurrent())),
            )
        )
        #return (self.leftShootingMotor.getOutputCurrent(), self.rightShootingMotor.getOutputCurrent())
