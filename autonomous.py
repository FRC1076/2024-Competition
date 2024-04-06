import wpilib
from dashboard import Dashboard
from wpimath.trajectory import Trajectory, TrajectoryUtil
from pathplannerlib.path import PathPlannerPath
from wpimath.kinematics import ChassisSpeeds
from wpimath.geometry import Rotation2d
from pathplannerlib.config import PIDConstants
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.telemetry import PPLibTelemetry
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
        self.noteDriveCounter = 0
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
        self.maxPickUpDistance = config['MAX_PICK_UP_DISTANCE']

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
            x = abs(self.autonTask[1])
            y = self.autonTask[2]
            bearing = self.autonTask[3]
            if not self.team_is_red:
                x = -x
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
            #set waitTime to -1 if you don't want to use note detection
            waitTime = self.autonTask[2]
            #when this command is first called
            if self.lastTime == -1:
                #update the time
                self.lastTime = self.autonTimer.get()
                #load the path, and flip depending on side
                self.path = PathPlannerPath.fromPathFile(self.autonTask[1])
                rotation = self.swervometer.getPathPlannerPose().rotation()
                if(self.team_is_red):
                    self.path = self.path.flipPath()
                self.pathTrajectory = self.path.getTrajectory(ChassisSpeeds(), rotation)
                #log the path the pathplanner's telemetry
                PPLibTelemetry.setCurrentPath(self.path)
                self.holonomicController.reset(self.swervometer.getPathPlannerPose(), ChassisSpeeds())
            #get the target state of the robot (pathState) and calculate the robot's chassis speeds
            self.pathState = self.pathTrajectory.sample(self.autonTimer.get() - self.lastTime)
            self.chassisSpeeds = self.holonomicController.calculateRobotRelativeSpeeds(self.swervometer.getPathPlannerPose(), self.pathState)
            #log the current pose and target pose to pathplanner's telemetry
            PPLibTelemetry.setCurrentPose(self.swervometer.getPathPlannerPose())
            PPLibTelemetry.setTargetPose(self.pathState.getTargetHolonomicPose())
            #log the path inaccuracy as a distance
            relativePose = self.pathState.getTargetHolonomicPose().relativeTo(self.swervometer.getPathPlannerPose())
            inaccuracyDistance = math.sqrt(pow(relativePose.X(), 2) + pow(relativePose.Y(), 2))
            PPLibTelemetry.setPathInaccuracy(inaccuracyDistance)

            self.noteDriveCounter += 1

            #if there is a note, it is within range, waitTime isn't negative, and the waitTime has passed, then use note detection
            if(self.notedetector.hasTarget() and waitTime is not None and self.notedetector.getTargetErrorY() < self.maxPickUpDistance and self.autonTimer.get() - self.lastTime > waitTime and not self.mechanism.indexBeamBroken()):
                #move every other robot cycle
                # if(self.noteDriveCounter % 2 == 0):
                #     # Use chassisSpeeds for forwards and rotational movement, but use notedetector for lateral movement
                #     self.moduleStates = self.swervometer.getKinematics().toSwerveModuleStates(ChassisSpeeds(self.chassisSpeeds[0], self.drivetrain.noteDrive_x_pid_controller.calculate(self.notedetector.getTargetErrorX()) * 4.3 * 10, self.chassisSpeeds[2]))
                # else:
                #     self.moduleStates = self.swervometer.getKinematics().toSwerveModuleStates(ChassisSpeeds(self.chassisSpeeds[0]/2, 0, 0))
                self.moduleStates = self.swervometer.getKinematics().toSwerveModuleStates(ChassisSpeeds(self.chassisSpeeds[0], self.drivetrain.noteDrive_x_pid_controller.calculate(self.notedetector.getTargetErrorX()) * 4.3, self.chassisSpeeds[2]))

                self.modules = self.drivetrain.getModules()
                self.modules['front_left'].move(self.moduleStates[0].speed / (self.maxSpeed), (self.moduleStates[0].angle.degrees() + 270) % 360)
                self.modules['front_right'].move(self.moduleStates[1].speed / (self.maxSpeed), (self.moduleStates[1].angle.degrees() + 270) % 360)
                self.modules['rear_left'].move(self.moduleStates[2].speed / (self.maxSpeed), (self.moduleStates[2].angle.degrees() + 270) % 360)
                self.modules['rear_right'].move(self.moduleStates[3].speed / (self.maxSpeed), (self.moduleStates[3].angle.degrees() + 270) % 360)
                self.modules['front_left'].execute()
                self.modules['front_right'].execute()
                self.modules['rear_left'].execute()
                self.modules['rear_right'].execute()
            
            #if the speeds are minimal and minimum time has elapsed, move onto the next task
            elif((abs(self.chassisSpeeds.vx/self.maxSpeed) < 0.1 and abs(self.chassisSpeeds.vy/self.maxSpeed) < 0.1 and abs(self.chassisSpeeds.omega_dps) < 5 and self.autonTimer.get() - self.lastTime > self.pathTrajectory.getTotalTimeSeconds()) or (self.mechanism.indexBeamBroken() and waitTime is not None)):
                self.moduleStates = self.swervometer.getKinematics().toSwerveModuleStates(ChassisSpeeds(0, 0, 0))
                self.modules = self.drivetrain.getModules()
                self.modules['front_left'].move(self.moduleStates[0].speed / (self.maxSpeed), (self.moduleStates[0].angle.degrees() + 270) % 360)
                self.modules['front_right'].move(self.moduleStates[1].speed / (self.maxSpeed), (self.moduleStates[1].angle.degrees() + 270) % 360)
                self.modules['rear_left'].move(self.moduleStates[2].speed / (self.maxSpeed), (self.moduleStates[2].angle.degrees() + 270) % 360)
                self.modules['rear_right'].move(self.moduleStates[3].speed / (self.maxSpeed), (self.moduleStates[3].angle.degrees() + 270) % 360)
                self.modules['front_left'].execute()
                self.modules['front_right'].execute()
                self.modules['rear_left'].execute()
                self.modules['rear_right'].execute()
                self.lastTime = -1
                self.taskListCounter += 1

            else:
                #drive the robot
                self.moduleStates = self.swervometer.getKinematics().toSwerveModuleStates(self.chassisSpeeds)
                self.modules = self.drivetrain.getModules()
                self.modules['front_left'].move(self.moduleStates[0].speed / (self.maxSpeed), (self.moduleStates[0].angle.degrees() + 270) % 360)
                self.modules['front_right'].move(self.moduleStates[1].speed / (self.maxSpeed), (self.moduleStates[1].angle.degrees() + 270) % 360)
                self.modules['rear_left'].move(self.moduleStates[2].speed / (self.maxSpeed), (self.moduleStates[2].angle.degrees() + 270) % 360)
                self.modules['rear_right'].move(self.moduleStates[3].speed / (self.maxSpeed), (self.moduleStates[3].angle.degrees() + 270) % 360)
                self.modules['front_left'].execute()
                self.modules['front_right'].execute()
                self.modules['rear_left'].execute()
                self.modules['rear_right'].execute()
        
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
                self.swervometer.enableVision()
            if(self.autonTimer.get() - self.lastTime > 0.25):
                #self.swervometer.disableVision()
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
            if(self.autonTimer.get() - self.lastTime > 1):
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

        elif self.autonTask[0] == 'MOVE_TO_NOTE':
            expectedX = abs(self.autonTask[1])
            if not self.team_is_red:
                expectedX *= -1
            expectedY = self.autonTask[2]
            bearing = self.autonTask[3]
            waitTime = self.autonTask[4]
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()

            if not self.drivetrain.goToPose(expectedX, expectedY, bearing):
                if self.notedetector.hasTarget() and self.notedetector.getTargetErrorY() < self.maxPickUpDistance and self.autonTimer.get() - self.lastTime > waitTime:
                    self.taskList.insert(self.taskListCounter + 1, ['PICK_UP_NOTE'])
                    self.lastTime = -1
                    self.taskListCounter += 1
            else:
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.lastTime = -1
                self.taskListCounter += 1
        
        elif self.autonTask[0] == 'PICK_UP_NOTE':
            if self.lastTime == -1:
                self.lastTime = self.autonTimer.get()
            self.drivetrain.alignWithNote(0, -3, None)
            if self.mechanism.indexBeamBroken():
                self.lastTime = -1
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.taskListCounter += 1
            elif self.autonTimer.get() - self.lastTime > 2:
                self.lastTime = -1
                self.drivetrain.set_fwd(0)
                self.drivetrain.set_strafe(0)
                self.drivetrain.set_rcw(0)
                self.drivetrain.execute('center')
                self.taskListCounter += 1
        
        elif self.autonTask[0] == "ENABLE_VISION":
            self.swervometer.enableVision()
            self.taskListCounter += 1

        elif self.autonTask[0] == "DISABLE_VISION":
            self.swervometer.disableVision()
            self.taskListCounter += 1
            
        return False

    def move(self):
        return