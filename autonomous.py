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

    def __init__(self, config, team_is_red, field_start_position, drivetrain, mechanism, swervometer):
        taskListName = config["TASK"]
        self.taskList = []
        for cmd in config[taskListName]:
            if cmd[0] == 'NOTE':
                for noteCmd in config["noteConfig"]["NOTE " + str(cmd[1])]:
                    self.taskList.append(noteCmd)
                pass
            else:
                self.taskList.append(cmd)
        self.taskListCounter = 0

        self.autonTimer = wpilib.Timer()
        self.autonHasStarted = False
        self.drivetrain = drivetrain
        self.mechanism = mechanism
        self.lastTime = -1
        self.hasRolledBack = False

        self.holonomicController = PPHolonomicDriveController(PIDConstants(2, 0, 0), PIDConstants(0, 0, 0), 3, 0.5388, 0.2)
        self.swervometer = swervometer
        self.team_is_red = team_is_red

    def executeAuton(self):
        print(self.taskListCounter)
        if not self.autonHasStarted:
            self.autonTimer.start()
            self.autonHasStarted = True

        if self.taskListCounter >= len(self.taskList):
            self.drivetrain.set_fwd(0)
            self.drivetrain.set_strafe(0)
            self.drivetrain.set_rcw(0)
            self.drivetrain.execute('center')
            print("tasks arae done")
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
        
        elif self.autonTask[0] == 'PATH':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
                self.path = PathPlannerPath.fromPathFile(self.autonTask[1])
                if(self.team_is_red):
                    self.path = self.path.flipPath()
                self.pathTrajectory = self.path.getTrajectory(ChassisSpeeds(), Rotation2d())
            self.pathState = self.pathTrajectory.sample(self.autonTimer.get() - self.lastTime)
            self.chassisSpeeds = self.holonomicController.calculateRobotRelativeSpeeds(self.swervometer.getPathPlannerPose(), self.pathState)
            self.drivetrain.set_fwd(-self.chassisSpeeds.vy/4)
            self.drivetrain.set_strafe(self.chassisSpeeds.vx/4)
            if self.drivetrain.shouldSteerStraight():
                if(self.team_is_red):
                    self.drivetrain.set_rcw(self.drivetrain.steerStraight(0, 0))
                else:
                    self.drivetrain.set_rcw(self.drivetrain.steerStraight(0, 180))
            if(abs(self.chassisSpeeds.vx/3) < 0.1 and abs(self.chassisSpeeds.vy/3) < 0.1 and self.autonTimer.get() - self.lastTime > self.pathTrajectory.getTotalTimeSeconds()):
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
            if self.hasRolledBack == False and (self.autonTimer.get() - self.lastTime < 0.5):
                self.mechanism.indexFixedRollBack()
                self.hasRolledBack = True
            if(self.autonTimer.get() - self.lastTime > 0.5):
                self.mechanism.setShootState(True)
                self.mechanism.indexNote()
            if(self.autonTimer.get() - self.lastTime > 1):
                self.mechanism.sprocketToPosition(-37)
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
            if self.mechanism.indexBeamBroken():
                self.mechanism.stopIndexing()
                self.mechanism.setAutonSprocketPosition(self.autonTask[1])
                self.taskListCounter += 1
            else:
                self.mechanism.indexNote()

        elif self.autonTask[0] == 'LOWER_ARM_START':
            self.mechanism.setAutonSprocketPosition(self.autonTask[1])
            self.taskListCounter += 1

        return False
    
    def move(self):
        return