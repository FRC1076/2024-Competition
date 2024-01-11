import wpilib
import wpilib.drive
import wpimath.controller
from wpimath.controller import PIDController
import rev
from wpilib import DoubleSolenoid
import ctre

class Mechanism:
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

    def transportNote(self,direction):
        #direction is up or down

        #L2 is up, R2 is down
        return