import math
import wpilib
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.geometry import Translation2d
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition
from wpimath.geometry import Pose2d
from collections import namedtuple

# Create the structure of the field config:
FieldConfig = namedtuple('FieldConfig', ['sd_prefix',
                                         'origin_x', 'origin_y',
                                         'start_position_x', 'start_position_y', 'start_angle'])

# Create the structure of the robot property config: 
RobotPropertyConfig = namedtuple('RobotPropertyConfig', ['sd_prefix',
                                                         'is_red_team',
                                                         'team_gyro_adjustment',
                                                         'team_move_adjustment',
                                                         'use_com_adjustment',
                                                         'frame_dimension_x', 'frame_dimension_y',
                                                         'bumper_dimension_x', 'bumper_dimension_y',
                                                         'cof_offset_x', 'cof_offset_y',
                                                         'com_offset_x', 'com_offset_y',
                                                         'gyro_offset_x', 'gyro_offset_y',
                                                         'camera_offset_x', 'camera_offset_y',
                                                         'swerve_module_offset_x', 'swerve_module_offset_y'])

class Swervometer:
    def __init__(self, field_cfg, robot_property_cfg):
        self.field = field_cfg
        self.robotProperty = robot_property_cfg
        self.teamGyroAdjustment=self.robotProperty.team_gyro_adjustment
        self.teamMoveAdjustment=self.robotProperty.team_move_adjustment
        self.useCOMadjustment = self.robotProperty.use_com_adjustment
        #print("field.orgin_x", self.field.origin_x, " field.origin_y", self.field.origin_y)
        #print("field.start_position_x: ", self.field.start_position_x, " field.start_position_y: ", self.field.start_position_y)
        #print("bumper_dimension_x: ", self.robotProperty.bumper_dimension_x, " bumper_dimension_y: ", self.robotProperty.bumper_dimension_y)
        #print("cof_offset_x: ", self.robotProperty.cof_offset_x, " cof_offset_y: ", self.robotProperty.cof_offset_y)
        self.currentX = self.field.start_position_x
        self.currentY = self.field.start_position_y
        self.currentBearing = self.field.start_angle
        self.swerveModuleOffsetX = self.robotProperty.swerve_module_offset_x
        self.swerveModuleOffsetY = self.robotProperty.swerve_module_offset_y
        self.frame_dimension_x = self.robotProperty.frame_dimension_x
        self.frame_dimension_y = self.robotProperty.frame_dimension_y
        self.com_offset_x = self.robotProperty.com_offset_x
        self.com_offset_y = self.robotProperty.com_offset_y
        self.starting_angle = self.field.start_angle
        #print("init current X: ", self.currentX, " init current y: ", self.currentY, " init current bearing: ", self.currentBearing)
        self.useVision = False
        self.looseVision = False

        self.chassisSpeeds = ChassisSpeeds()
        self.lastDistancesX = []
        self.lastDistancesY = []

        self.calcLeverArmLengths()

    def startTimer(self):
        self.timer = wpilib.Timer()
        self.timer.start()

    def getTimer(self):
        return self.timer.get()

    def getFrameDimensions(self):
        #return self.frame_dimension_x, self.frame_dimension_y
        return self.swerveModuleOffsetX * 2, self.swerveModuleOffsetY * 2

    def getTeamGyroAdjustment(self):
        return self.teamGyroAdjustment

    def getTeamMoveAdjustment(self):
        return self.teamMoveAdjustment

    def getCOF(self):
        return self.currentX, self.currentY, self.currentBearing
    
    def getPathPlannerPose(self):
        #if (-(self.currentBearing + 180)) % 360 < 180:
        return Pose2d(self.currentX * 0.0254 + 8.28, self.currentY * 0.0254 + 4.10, Rotation2d.fromDegrees((-(self.currentBearing + 180)) % 360)) #Rotation2d.fromDegrees(180))#
        #else:
            #return Pose2d(self.currentX * 0.0254 + 8.28, self.currentY * 0.0254 + 4.10, Rotation2d.fromDegrees(((-(self.currentBearing + 180)) % 360) - 360))

    def setCOF(self, x, y, bearing):
        #self.currentX = x
        #self.currentY = y
        #self.currentBearing = bearing
        pass

    def calcLeverArmLengths(self):

        if self.useCOMadjustment:
            
            # Calc X and Y distances from COM
            frontLeverArmX = self.swerveModuleOffsetX - self.com_offset_x
            rearLeverArmX = self.swerveModuleOffsetX + self.com_offset_x
            leftLeverArmY = self.swerveModuleOffsetY + self.com_offset_y
            rightLeverArmY = self.swerveModuleOffsetY - self.com_offset_y
            #print("FL: ", frontLeverArmX, " RL: ", rearLeverArmX, " LL: ", leftLeverArmY, " RL: ", rightLeverArmY)

            # Calc true lever lengths
            frontLeftLeverLength = math.hypot(frontLeverArmX, leftLeverArmY)
            frontRightLeverLength = math.hypot(frontLeverArmX, rightLeverArmY)
            rearLeftLeverLength = math.hypot(rearLeverArmX, leftLeverArmY)
            rearRightLeverLength = math.hypot(rearLeverArmX, rightLeverArmY)

            # Calc average lever length
            avgLeverLength = (frontLeftLeverLength + frontRightLeverLength + rearLeftLeverLength + rearRightLeverLength) / 4

            # Normalize lever lengths
            self.frontLeftCOMmult = avgLeverLength / frontLeftLeverLength
            self.frontRightCOMmult = avgLeverLength / frontRightLeverLength
            self.rearLeftCOMmult = avgLeverLength / rearLeftLeverLength
            self.rearRightCOMmult = avgLeverLength / rearRightLeverLength

        else:
            self.frontLeftCOMmult = 1.0
            self.frontRightCOMmult = 1.0
            self.rearLeftCOMmult = 1.0
            self.rearRightCOMmult = 1.0

    def getCOMmult(self, key): 
        if (key == 'front_right'):
            return self.frontRightCOMmult
        elif (key == 'rear_right'):
            return self.rearRightCOMmult
        elif (key == 'rear_left'):
            return self.rearLeftCOMmult
        else: # (key == 'front_left'):
            return self.frontLeftCOMmult
    """
    def calculateModuleCoordinates(self, psi, currentGyroAngle, hypotenuse, positionChange, wheelAngle):
        #print("calcModCoord: psi: ", psi, " currentGyroAngle: ", currentGyroAngle, " hypo: ", hypotenuse, " posChg: ", positionChange, " wheelAngle: ", wheelAngle)
        
        baseAngle = (psi + currentGyroAngle) % 360 # angle of the module
        swerveModuleOffsetXCoordinate = hypotenuse * math.cos(math.radians(baseAngle)) # X-position of the module
        swerveModuleOffsetYCoordinate = hypotenuse * math.sin(math.radians(baseAngle)) # Y-position of the module
        #print("baseAngle: ", baseAngle, " swerveModuleOffsetXCoordinate: ", swerveModuleOffsetXCoordinate, " swerveModuleOffsetYCoordinate: ", swerveModuleOffsetYCoordinate)

        combinedAngle = (currentGyroAngle + wheelAngle) % 360 # angle of the wheel
        XChange = positionChange * math.cos(math.radians(combinedAngle)) # change in X-position of the module
        YChange = positionChange * math.sin(math.radians(combinedAngle)) # change in Y-position of the module
        #print("combinedAngle: ", combinedAngle, "sin(rad(combinedAngle)): ", math.sin(math.radians(combinedAngle)), "cos(rad(combinedAngle)): ", math.cos(math.radians(combinedAngle)), " XChange: ", XChange, " YChange: ", YChange)

        XCoordinate = self.currentX + swerveModuleOffsetXCoordinate + XChange # current X-coordinate of COF plus swerve module offset plus movement
        YCoordinate = self.currentY + swerveModuleOffsetYCoordinate + YChange # current Y-coordinate of COF plus swerve module offset plus movement
        #print("XCoordinate: ", XCoordinate, " YCoordinate: ", YCoordinate)

        return XCoordinate, YCoordinate

    def calculateCOFPose(self, modules, currentGyroAngle):

        # The bot is assumed to orient so that "0 degrees" faces "north" along the Y-axis of the frame in the forward direction.
        
        # currentGyroAngle is the clockwise rotation of the bot as determined by the gyro.

        # psi is the fixed angle from the front of the bot to the front right corner.
        # Mod 360 shouldn't be needed.
        # Although we recalculate it here, each psi and the hypotenuse are constants.

        frontRightPsi = math.degrees(math.atan(self.swerveModuleOffsetY / self.swerveModuleOffsetX)) % 360
        rearRightPsi = (frontRightPsi + 90) % 360
        rearLeftPsi = (frontRightPsi + 180) % 360
        frontLeftPsi = (frontRightPsi + 270) % 360
        hypotenuse = math.sqrt((self.swerveModuleOffsetX ** 2) + (self.swerveModuleOffsetY ** 2))

        #print("hypotenuse: ", hypotenuse)

        for key in modules:
            # positionChange is the amount the wheel moved forward
            positionChange = modules[key].positionChange

            # wheelAngle is the angle of the module wheel relative to the frame of the bot
            wheelAngle = (modules[key].newAngle - 90) % 360 # The -90 is because the orientation of the swervemodules seems to be negative 90 degrees off from the orientation of the bot.
            
            # Each of these calculations is different because positionChange, newAngle, and psi are different for each corner
            if (key == 'front_right'):
                frontRightXCoordinate, frontRightYCoordinate = self.calculateModuleCoordinates(frontRightPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                #print("fr: pc: ", positionChange, " psi: ", frontRightPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "frx: ", frontRightXCoordinate, "fry: ", frontRightYCoordinate)
            elif (key == 'rear_right'):
                rearRightXCoordinate, rearRightYCoordinate = self.calculateModuleCoordinates(rearRightPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                #print("rr: pc: ", positionChange, " psi: ", rearRightPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "rrx: ", rearRightXCoordinate, "rry: ", rearRightYCoordinate)
            elif (key == 'rear_left'):
                rearLeftXCoordinate, rearLeftYCoordinate = self.calculateModuleCoordinates(rearLeftPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                #print("rl: pc: ", positionChange, " psi: ", rearLeftPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "rlx: ", rearLeftXCoordinate, "rly: ", rearLeftYCoordinate)
            else: # (key == 'front_left'):
                frontLeftXCoordinate, frontLeftYCoordinate = self.calculateModuleCoordinates(frontLeftPsi, currentGyroAngle, hypotenuse, positionChange, wheelAngle)
                #print("fl: pc: ", positionChange, " psi: ", frontLeftPsi, " bot angle: ", currentGyroAngle, " wheel angle: ", wheelAngle, "flx: ", frontLeftXCoordinate, "fly: ", frontLeftYCoordinate)
            
        # Find average COF XY-coordinates of bot
        midpointX1 = (frontLeftXCoordinate + rearRightXCoordinate)/2
        midpointY1 = (frontLeftYCoordinate + rearRightYCoordinate)/2

        midpointX2 = (frontRightXCoordinate + rearLeftXCoordinate)/2
        midpointY2 = (frontRightYCoordinate + rearLeftYCoordinate)/2

        midpointX = (midpointX1 + midpointX2)/2
        midpointY = (midpointY1 + midpointY2)/2

        #print("old x: ", self.currentX, " old y: ", self.currentY)
        #print("draft x1:", midpointX1, "draft x2:", midpointX2, "draft x: ", midpointX, "draft y1:", midpointY1, "draft y2:", midpointY2, " draft y: ", midpointY)

        # Reset pose of bot.
        self.currentX = midpointX
        self.currentY = midpointY
        self.currentBearing = currentGyroAngle
        
        return self.currentX, self.currentY, self.currentBearing
    """

    def getKinematics(self):
        return self.kinematics
    
    def enableVision(self):
        self.useVision = True

    def disableVision(self):
        self.useVision = False
    
    def enableLooseVision(self):
        self.looseVision = True
    
    def disableLooseVision(self):
        self.looseVision = False

    def initPoseEstimator(self, modules, vision):
        frontLeftModule = modules['front_left'].getSwerveModulePosition()
        frontRightModule = modules['front_right'].getSwerveModulePosition()
        rearLeftModule = modules['rear_left'].getSwerveModulePosition()
        rearRightModule = modules['rear_right'].getSwerveModulePosition()
        #correct order kinematics
        self.kinematics = SwerveDrive4Kinematics(Translation2d(self.swerveModuleOffsetX * 0.0254, self.swerveModuleOffsetY * 0.0254),
                                             Translation2d(self.swerveModuleOffsetX * 0.0254, -self.swerveModuleOffsetY * 0.0254),
                                             Translation2d(-self.swerveModuleOffsetX * 0.0254, self.swerveModuleOffsetY * 0.0254),
                                             Translation2d(-self.swerveModuleOffsetX * 0.0254, -self.swerveModuleOffsetY * 0.0254))
        gyroAngle = Rotation2d().fromDegrees(self.teamGyroAdjustment)
        swerveModules = (frontLeftModule, frontRightModule, rearLeftModule, rearRightModule)
        self.poseEstimator =  SwerveDrive4PoseEstimator(self.kinematics, gyroAngle, swerveModules, Pose2d(self.currentX * 0.0254, self.currentY * 0.0254, self.teamGyroAdjustment * math.pi / 180), (0.1, 0.1, 0.1), (1.8, 1.8, 1.8))
        self.vision = vision
        self.currentBearing = self.teamGyroAdjustment

    """
    Take in our coordinate system
    """
    def resetPoseEstimator(self, gyroAngle, x, y, modules):
        #Pose2d(self.currentX * 0.0254 + 8.28, self.currentY * 0.0254 + 4.10, Rotation2d.fromDegrees((-(self.currentBearing + 180)) % 360))
        #assume it starts straight for now
        frontLeftModule = modules['front_left'].getSwerveModulePosition()
        frontRightModule = modules['front_right'].getSwerveModulePosition()
        rearLeftModule = modules['rear_left'].getSwerveModulePosition()
        rearRightModule = modules['rear_right'].getSwerveModulePosition()
        self.poseEstimator.resetPosition(Rotation2d.fromDegrees((-(gyroAngle)) % 360), (frontLeftModule, frontRightModule, rearLeftModule, rearRightModule), Pose2d(x * 0.0254, y * 0.0254, self.teamGyroAdjustment * math.pi / 180))
        self.currentX = x
        self.currentY = y
        self.currentBearing = gyroAngle

    def updatePoseEstimator(self, gyroAngle, modules, gyroRate = 0):
        frontLeftModule = modules['front_left'].getSwerveModulePosition()
        frontRightModule = modules['front_right'].getSwerveModulePosition()
        rearLeftModule = modules['rear_left'].getSwerveModulePosition()
        rearRightModule = modules['rear_right'].getSwerveModulePosition()
        #self.currentPose = self.poseEstimator.updateWithTime(self.getTimer(), Rotation2d(gyroAngle * math.pi / 180), (frontLeftModule, frontRightModule, rearLeftModule, rearRightModule))
        self.currentBearing = gyroAngle
        #self.currentPose = self.poseEstimator.updateWithTime(self.getTimer(), Rotation2d(gyroAngle * math.pi / 180), (frontLeftModule, frontRightModule, rearLeftModule, rearRightModule))
        self.currentPose = self.poseEstimator.update(Rotation2d.fromDegrees((-(self.currentBearing)) % 360), (frontLeftModule, frontRightModule, rearLeftModule, rearRightModule))
        #print(self.useVision)
        #set the robot orientation for the limelight
        self.vision.setYawOrientation((-(gyroAngle) + 180) % 360, 0)#(-gyroRate) % 360)
        if(self.vision.hasTargets() and self.useVision):
            try:
                #print("POSE UPDATING AHHHHHHHHHHH")
                pose = self.vision.getMegatag2Pose()
                self.poseEstimator.addVisionMeasurement(Pose2d(pose[0] * 0.0254, pose[1] * 0.0254, Rotation2d.fromDegrees((-(gyroAngle)) % 360)), wpilib.Timer.getFPGATimestamp() - self.vision.getTotalLatency() / 1000)
            except:
                pass
        
        self.currentPose = self.poseEstimator.getEstimatedPosition()
        self.currentX = self.currentPose.X() * 39.37
        self.currentY = self.currentPose.Y() * 39.37

        self.chassisSpeeds = self.kinematics.toChassisSpeeds((modules['front_left'].getSwerveModuleState(), modules['front_right'].getSwerveModuleState(), modules['rear_left'].getSwerveModuleState(), modules['rear_right'].getSwerveModuleState()))
        self.lastDistancesX.append(-self.chassisSpeeds.vy * 39.37 * 0.02)
        self.lastDistancesY.append(self.chassisSpeeds.vx * 39.37 * 0.02)

        if len(self.lastDistancesX) > 20:
            self.lastDistancesX.pop(0)
        if len(self.lastDistancesY) > 20:
            self.lastDistancesY.pop(0)

    def getChassisSpeeds(self):
        return self.chassisSpeeds
    
    def getDistanceTraveledX(self, botCycles):
        return sum(self.lastDistancesX[max(0, len(self.lastDistancesX) - 1 - botCycles):]) 
    
    def getDistanceTraveledY(self, botCycles):
        return sum(self.lastDistancesY[max(0, len(self.lastDistancesY) - 1 - botCycles):])
        #print("CURRENT POSE", self.currentX, self.currentY)
        
    def distanceToPose(self, x, y):
        return math.sqrt(pow(x - self.currentX, 2) + pow(y - self.currentY, 2))
