import wpilib
from dashboard import Dashboard
from wpimath.trajectory import Trajectory, TrajectoryUtil
from pathplannerlib.path import PathPlannerPath
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
import math

class Autonomous:

    def __init__(self, config, team_is_red, field_start_position, drivetrain, mechanism, notedetector, swervometer, starting_angle, taskListName):
        self.taskList = []
        for cmd in config[taskListName]:
            if cmd[0] == 'NOTE':
                for noteCmd in config["noteConfig"]["NOTE " + str(cmd[1])]:
                    self.taskList.append(noteCmd)
                pass
            else:
                self.taskList.append(cmd)
        self.taskListCounter = 0

        self.maxSpeed = config["MAX_SPEED_M/S"]

        self.autonTimer = wpilib.Timer()
        self.autonHasStarted = False
        self.drivetrain = drivetrain
        self.mechanism = mechanism
        self.notedetector = notedetector
        self.lastTime = -1
        self.hasRolledBack = False

        self.holonomicController = PPHolonomicDriveController(
                                    PIDConstants(config['TRANSLATION_KP'], config['TRANSLATION_KI'], config['TRANSLATION_KD']), 
                                    PIDConstants(config['ROTATION_KP'], config['ROTATION_KI'], config['ROTATION_KD']), 
                                    config['MAX_SPEED_M/S'], 
                                    config['DRIVE_BASE_RADIUS'], 
                                    config['PERIOD'])
        self.swervometer = swervometer
        self.currentX, self.currentY, self.currentBearing = self.swervometer.getCOF()
        self.team_is_red = team_is_red

        self.maxSpeed = config['MAX_SPEED_M/S']

    def executeAuton(self):
        if not self.autonHasStarted:
            self.autonTimer.start()
            self.autonHasStarted = True

        if self.taskListCounter >= len(self.taskList):
            self.drivetrain.set_fwd(0)
            self.drivetrain.set_strafe(0)
            self.drivetrain.set_rcw(0)
            self.drivetrain.execute('center')
            #print("tasks arae done")
            return False
        
        self.autonTask = self.taskList[self.taskListCounter]

        if self.autonTask[0] == "MOVE":
            x = self.autonTask[1]
            y = self.autonTask[2]
            bearing = self.autonTask[3]
            if self.drivetrain.goToPose(x, y, bearing):
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.taskListCounter += 1 # Move on to next task.
            return True
        
        elif self.autonTask[0] == 'MOVE_RELATIVE':
            x = self.autonTask[1]
            y = self.autonTask[2]
            bearing = self.autonTask[3]
            if self.drivetrain.goToRelativePose(x, y, bearing):
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.taskListCounter += 1
            return True
        
        elif self.autonTask[0] == 'PATH':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
                self.path = PathPlannerPath.fromPathFile(self.autonTask[1])
                if(self.team_is_red):
                    self.path = self.path.flipPath()
                self.pathTrajectory = self.path.getTrajectory(ChassisSpeeds(), Rotation2d())
            self.pathState = self.pathTrajectory.sample(self.autonTimer.get() - self.lastTime)
            self.chassisSpeeds = self.holonomicController.calculateRobotRelativeSpeeds(self.swervometer.getPathPlannerPose(), self.pathState)
            self.drivetrain.set_fwd(-self.chassisSpeeds.vy/self.maxSpeed)
            self.drivetrain.set_strafe(self.chassisSpeeds.vx/self.maxSpeed)
            if self.drivetrain.shouldSteerStraight():
                if(self.team_is_red):
                    self.drivetrain.set_rcw(self.drivetrain.steerStraight(0, 0))
                else:
                    self.drivetrain.set_rcw(self.drivetrain.steerStraight(0, 180))
            if(abs(self.chassisSpeeds.vx/self.maxSpeed) < 0.1 and abs(self.chassisSpeeds.vy/self.maxSpeed) < 0.1 and self.autonTimer.get() - self.lastTime > self.pathTrajectory.getTotalTimeSeconds()):
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.lastTime = -1
                self.taskListCounter += 1
            self.drivetrain.execute('center')
        
        elif self.autonTask[0] == 'WHEEL_LOCK':           
            self.drivetrain.setWheelLock(True)
            self.drivetrain.move(0, 0, 0, self.drivetrain.getBearing())
            self.drivetrain.execute('center')
            #self.taskListCounter += 1 # Move on to next task.
        
        elif self.autonTask[0] == 'WAIT':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
            if(self.autonTimer.get() - self.lastTime > self.autonTask[1]):
                self.lastTime = -1
                self.taskListCounter += 1
        
        elif self.autonTask[0] == 'UPDATE_POSE':
            #self.drivetrain.visionUpdatePose()
            self.taskListCounter += 1
        
        elif self.autonTask[0] == 'START_INTAKE':
            self.mechanism.intakeNote()
            self.mechanism.shootNote()
            self.taskListCounter += 1
        
        elif self.autonTask[0] == 'STOP_INTAKE':
            self.mechanism.stopIntake()
            self.mechanism.stopIndexing()
            self.taskListCounter += 1

        elif self.autonTask[0] == 'SHOOT_NOTE':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
                self.mechanism.setShootState(True)
                self.mechanism.indexNote()
            if(self.autonTimer.get() - self.lastTime > 0.25):
                #self.mechanism.sprocketToPosition(-37)
                self.mechanism.setShootState(False)
                self.mechanism.stopIndexing()
                self.lastTime = -1
                self.taskListCounter += 1
        
        elif self.autonTask[0] == 'RAISE_ARM': 
            if self.mechanism.indexBeamBroken():
                self.mechanism.stopIndexing()
            else:
                self.mechanism.indexNote()
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
            if(self.autonTimer.get() - self.lastTime > 1):
                if self.mechanism.sprocketToPosition(self.autonTask[1]):
                    self.mechanism.stopSprocket()
                    self.lastTime = -1
                    self.taskListCounter += 1

        elif self.autonTask[0] == 'LOWER_ARM':
            if self.mechanism.sprocketToPosition(self.autonTask[1]):
                self.mechanism.stopIndexing()
                self.mechanism.stopSprocket()
                self.taskListCounter += 1
        
        elif self.autonTask[0] == 'RAISE_ARM_START':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
            if(self.autonTimer.get() - self.lastTime > 2.5):
                self.mechanism.stopIndexing()
                self.mechanism.setAutonSprocketPosition(self.autonTask[1])
                self.lastTime = -1
                self.taskListCounter += 1   
            elif self.mechanism.indexBeamBroken():
                self.mechanism.stopIndexing()
                self.mechanism.setAutonSprocketPosition(self.autonTask[1])
                self.lastTime = -1
                self.taskListCounter += 1
            else:
                self.mechanism.indexNote()

        elif self.autonTask[0] == 'LOWER_ARM_START':
            self.mechanism.setAutonSprocketPosition(self.autonTask[1])
            self.taskListCounter += 1
        
        elif self.autonTask[0] == 'ROTATE':
            if self.team_is_red:
                if(self.drivetrain.rotateToAngle(self.autonTask[1])):
                    self.drivetrain.set_rcw(0)
                    self.taskListCounter += 1
            else:
                if(self.drivetrain.rotateToAngle(180 - self.autonTask[1])):
                    self.drivetrain.set_rcw(0)
                    self.taskListCounter += 1
            self.drivetrain.execute('center')

        elif self.autonTask[0] == 'POINT_TO_NOTE':
            self.goToNote = self.autonTask[1]
            self.backupTask = self.autonTask[2]
            if self.notedetector.hasTarget():
                #print(self.notedetector.getTargetErrorX())
                if abs(self.notedetector.getTargetErrorX()) < 2.0:
                    if self.goToNote:
                        #self.taskList.insert(self.taskListCounter + 1, ['MOVE', self.notedetector.getTargetErrorX(), self.notedetector.getTargetErrorY() - 5, self.notedetector.getTargetErrorAngle()])
                        pass
                    self.taskListCounter += 1 
                else:
                    self.drivetrain.alignWithNote(False, False, 0)
            else:
                self.taskList.insert(self.taskListCounter + 1, self.backupTask)
                self.taskListCounter += 1

        elif self.autonTask[0] == 'MOVE_TO_NOTE':
            expectedX = self.autonTask[1]
            expectedY = self.autonTask[2]
            bearing = self.autonTask[3]
            waitTime = self.autonTask[4]
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()

            if not self.drivetrain.goToPose(expectedX, expectedY, bearing):
                if self.notedetector.hasTarget() and self.notedetector.getTargetErrorY() < 45 and self.autonTimer.get() - self.lastTime > waitTime:
                    self.taskList.insert(self.taskListCounter + 1, ['PICK_UP_NOTE'])
                    self.taskListCounter += 1
            else:
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.taskListCounter += 1
        
        elif self.autonTask[0] == 'PICK_UP_NOTE':
            self.drivetrain.alignWithNote(0, 0, False)
            if self.mechanism.indexBeamBroken():
                self.taskListCounter += 1
            
        return False
    
    def move(self):
        return