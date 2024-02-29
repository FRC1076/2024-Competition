from collections import namedtuple
import math
DEADZONE = 0.1

ARCADE = 1
TANK = 2
SWERVE = 3

controllerConfig = {
    "DRIVER": {
        "ID": 0,
        "DEADZONE": DEADZONE,
        "LEFT_TRIGGER_AXIS": 2,
        "RIGHT_TRIGGER_AXIS": 3,
    },
    "OPERATOR": {
        "ID": 1,
        "DEADZONE": DEADZONE,
        "LEFT_TRIGGER_AXIS": 2,
        "RIGHT_TRIGGER_AXIS": 3,
    }
}

swervometerConfig = { # All positions measured in inches
    'TEAM_IS_RED': False, # Is the robot part of the Red Team?
    'FIELD_START_POSITION': 'C', # Which of three starting positions is selected?
    'FIELD_START_POSITION': 'D',
    'HAS_BUMPERS_ATTACHED': True, # Does the robot currently have bumpers attached?
    'USE_COM_ADJUSTMENT': True, # Should robot compensate for CoM lever arms?
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_RED_A_START_POSITION_X': 300, #values of 2024 on left #248.625, #159.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 104, #values of 2024 on left #16.75, #40.15, #54.25, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_A_START_ANGLE': 60, # Heading angle of starting position A when on red team
    'FIELD_RED_B_START_POSITION_X': 275, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': 57, #-28.25, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_B_START_ANGLE': 0, # Heading angle of starting position B when on red team
    'FIELD_RED_C_START_POSITION_X': 300, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': 10, #-137.90, # Y-Coordinate of starting postion C when on red team
    'FIELD_RED_C_START_ANGLE': 300, # Heading angle of starting position C when on red team
    'FIELD_BLU_A_START_POSITION_X': -300, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 104, # 40.15, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_A_START_ANGLE': 120, # Heading angle of starting position A when on blue team
    'FIELD_BLU_B_START_POSITION_X': -275, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': 57, #-28.25, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_B_START_ANGLE': 180.0, # Heading angle of starting position B when on blue team
    'FIELD_BLU_C_START_POSITION_X': -300, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': 10, # -137.90, # Y-Coordinate of starting postion C when on blue team
    'FIELD_BLU_C_START_ANGLE': 240.0, # Heading angle of starting position C when on blue team
    'FIELD_RED_D_START_POSITION_X': 300,
    'FIELD_RED_D_START_POSITION_Y': 114,
    'FIELD_RED_D_START_POSITION_ANGLE': 0,
    'FIELD_BLU_D_START_POSITION_X': -300,
    'FIELD_BLU_D_START_POSITION_Y': 114,
    'FIELD_BLU_D_START_ANGLE': 0,
    'ROBOT_FRAME_DIMENSION_X': 30.0, # X-coordinate length of robot frame  (2023: 34.0)
    'ROBOT_FRAME_DIMENSION_Y': 30.0, # Y-coordinate length of robot frame  (2023: 26.0)
    'ROBOT_BUMPER_DIMENSION_X': 3.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 3.0, # Width of bumper (Y-axis)
    'ROBOT_COF_OFFSET_X': 15.0, # X-offset of center of frame (assume half frame dimension) (2023: 17.0)
    'ROBOT_COF_OFFSET_Y': 15.0, # Y-offset of center of frame (assume half frame dimension) (2023: 13.0)
    'ROBOT_COM_OFFSET_X': -0.75, # X-offset of center of mass (relative to center of frame)
    'ROBOT_COM_OFFSET_Y': -0.75, # Y-offset of center of mass (relative to center of frame)
    'ROBOT_GYRO_OFFSET_X': 18.5, # X-offset of center of gyro (relative to lower left frame) 18.5 (2023: 15.0)
    'ROBOT_GYRO_OFFSET_Y': 4.25, # Y-offset of center of gyro (relative to lower left frame) 4.25 (2023: 12.0)
    'ROBOT_CAMERA_OFFSET_X': 17.0, # X-offset of center of camera lens (relative to center of frame)
    'ROBOT_CAMERA_OFFSET_Y': 0.0, # Y-offset of center of camera lens (relative to center of frame)
    'ROBOT_CAMERA_HEIGHT': 12.1875, # Height of camera eye relative to gyroscope: 11 3/16+ 2 -1
    'ROBOT_SWERVE_MODULE_OFFSET_X': 11.75, # X-offset of swerve module center from COF  (2023: 13.75)
    'ROBOT_SWERVE_MODULE_OFFSET_Y': 9.75, # Y-offset of swerve module center from COF   (2023: 9.75)
}

drivetrainConfig = {
    'FRONTLEFT_DRIVEMOTOR': 1,
    'FRONTRIGHT_DRIVEMOTOR': 2,
    'REARRIGHT_DRIVEMOTOR': 3,
    'REARLEFT_DRIVEMOTOR': 4,
    'FRONTLEFT_ROTATEMOTOR': 11,
    'FRONTRIGHT_ROTATEMOTOR': 12,
    'REARRIGHT_ROTATEMOTOR': 13,
    'REARLEFT_ROTATEMOTOR': 14,
    'FRONTLEFT_ENCODER': 21,
    'FRONTRIGHT_ENCODER': 22,
    'REARRIGHT_ENCODER': 23,
    'REARLEFT_ENCODER': 24,
    'DRIVETYPE': SWERVE,
    'HEADING_KP': 0.005, #0.005 - reverted to this
    'HEADING_KI': 0.00001, #0.00001 - reverted to this
    'HEADING_KD':  0.00001, #0.00001 - reverted to this
    'BALANCE_PITCH_KP': 0.01, #0.02, #0.01
    'BALANCE_PITCH_KI': 0.00001, #0.01, #0.00001
    'BALANCE_PITCH_KD':  0.0005, #0.0001, #0.0005
    'BALANCE_YAW_KP': 0.005,
    'BALANCE_YAW_KI': 0.00001,
    'BALANCE_YAW_KD': 0.00001,
    'TARGET_KP': 0.020,
    'TARGET_KI': 0.00, #0.005,
    'TARGET_KD': 0.0001,
    'BEARING_KP': 0.035,
    'BEARING_KI': 0.0,
    'BEARING_KD': 0.000,
    'ROBOT_INCHES_PER_ROTATION': 1.0, #1.793, # Inches per rotation of wheels
    'TELEOP_OPEN_LOOP_RAMP_RATE': 0.125, # Improves maneuverability of bot.
    'TELEOP_CLOSED_LOOP_RAMP_RATE': 0.125,
    'AUTON_STEER_STRAIGHT': True,
    'TELEOP_STEER_STRAIGHT': False,
    'ROTATE_CLOCKWISE': [['ROTATE', 179]], # 179, not -180 to ensure direction
    'ROTATE_COUNTERCLOCKWISE': [['ROTATE', -179]], # -179, not -180, to ensure direction
    'X_VISION_DRIVE_KP': 0.01, # 0.005,
    'X_VISION_DRIVE_KI': 0.000, #0.0006, #0.0045, # 0.00001,
    'X_VISION_DRIVE_KD': 0.0, # 0.0005,
    'Y_VISION_DRIVE_KP': 0.01, #0.005, # 0.0125,
    'Y_VISION_DRIVE_KI': 0, #0.0001, #0.0006, #0.0045, #0.0, # 0.00001,
    'Y_VISION_DRIVE_KD': 0.000,
    'R_VISION_DRIVE_KP': 0.003, # 0.0005,
    'R_VISION_DRIVE_KI': 0.00, #0.0006, #0.0045, # 0.00001,
    'R_VISION_DRIVE_KD': 0, #0.0005, # 0.0005,
    'REFLECTIVE_TARGET_TARGET_SIZE': 0.546, # 0.546% of the total field of view
    'REFLECTIVE_TARGET_OFFSET_X': -17.53,
    'APRIL_TARGET_TARGET_SIZE': 0.546, # % of the total field of view
    'APRIL_TARGET_OFFSET_X': -17.53,
    'MAX_TARGET_OFFSET_X': 90,
    'MIN_TARGET_SIZE': 0,
}

visionConfig = {
    'CAMERA_HEIGHT_FROM_GROUND': 18,
    'CAMERA_DISTANCE_FROM_COF': 12,
    'CAMERA_PITCH': 0,
    'APRILTAGS': 0,
    'RETROREFLECTIVE': 1,
    'MIN_TARGET_ASPECT_RATIO_REFLECTIVE': 0.0,
    'MAX_TARGET_ASPECT_RATIO_REFLECTIVE': 100.0,
    'MIN_TARGET_ASPECT_RATIO_APRILTAG': 0.0,
    'MAX_TARGET_ASPECT_RATIO_APRILTAG': 100.0,
    'UPDATE_POSE': False, # True if should correct position with Limelight information. Otherwise informational.
}

autonConfig = {
    'SCORE_EXISTING': True,
    'BALANCE_BOT': True,
    'DO_COMMUNITY': False, # Only applies for position B
    'AUTON_OPEN_LOOP_RAMP_RATE': 0.5, # Improves the quality of swervometery by avoiding slippage.
    'AUTON_CLOSED_LOOP_RAMP_RATE': 0,
    'TASK': 'C_ONE_NOTE',
    'B_FOUR_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'SHOT-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3'], ['RAISE_ARM_START', -23], ['PATH', '3-B'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    'B_THREE_NOTE_STAGE_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3'], ['RAISE_ARM_START', 0], ['PATH', '3-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    'B_THREE_NOTE_AMP_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'SHOT-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    'B_ONE_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-8']],
    'A_ONE_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['WAIT', 10], ['PATH', 'A-LEAVE']],
    'C_ONE_NOTE':[['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'C-8-ROTATE']],
    
    #'A_THREE_NOTE_AMP_SIDE_CENTER_LINE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'A-5']],
    #A_THREE_NOTE_AMP_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'A-1'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'SHOT-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    'C_TWO_NOTE_STAGE_SIDE_CENTER_LINE':[['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'C-8-ROTATE'], ['RAISE_ARM_START', -25.9], ['PATH', '8-C-ROTATE'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    
    # #RED TEAM,
    #     'NOTE 1': [['WAIT', 0], ['UPDATE_POSE'], ['MOVE', 216.5, 57, 0], ['MOVE', 275, 57, 0]],
    #     'NOTE 2': [['WAIT', 0], ['UPDATE_POSE'], ['MOVE', 240.25, 114, 0], ['MOVE', 216.5, 114, 0], ['MOVE', 275, 57, 0] ],#[['WAIT', 1], ['UPDATE_POSE'], ['MOVE', 262.25, 114, 0], ['MOVE', 230.5, 114, 0]],
    #     'NOTE 3': [['WAIT', 0], ['UPDATE_POSE'], ['MOVE', 240.25, 0, 0], ['MOVE', 216.5, 0, 0], ['MOVE', 275, 57, 0]],
    #     'NOTE 4': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', 115.31, -75, 0], ['MOVE', 19.9, -75, 0] ],
    #     'NOTE 5': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', 115.31, -34.31, 0],['MOVE', 19.9, -9, 0]],
    #     'NOTE 6 NEGATIVE': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', 115.31, -34.31, 0], ['MOVE', 19.9, 57, 0]],
    #     'NOTE 6 POSITIVE': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', 115.31, 148.31, 0], ['MOVE', 19.9, 57,0]],
    #     'NOTE 7': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', 115.31, 148.31, 0], ['MOVE', 19.9, 123, 0]],
    #     'NOTE 8': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', 115.31, 189, 0], ['MOVE', 19.9, 189, 0]],
    #     #BLUE TEAM
    #     'NOTE 9': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -207.03, 57, 180]],
    #     'NOTE 10': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -216.875, 114, 180], ['MOVE', -207.03, 114, 180]],
    #     'NOTE 11': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -216.875, 0, 180], ['MOVE', -207.03, 0, 180]],
    #     'NOTE 12': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, -75, 180], ['MOVE', 19.9, -75, 180]],
    #     'NOTE 13': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, -34.31, 180], ['MOVE', 19.9, -9, 180]],
    #     'NOTE 14 NEGATIVE': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, -34.41, 180], ['MOVE', 19.9, 57, 180]],
    #     'NOTE 14 POSITIVE': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, 148.31, 180], ['MOVE', 19.9, 57, 180]],
    #     'NOTE 15': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, 148.31, 180], ['MOVE', 19.9, 123, 180]],
    #     'NOTE 16': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, 189, 180], ['MOVE', 19.9, 189, 180]], 
    # },
}


MODULE_NAMES = namedtuple('MODULE_NAMES', [
    'ROBOT',
    'SWERVEDRIVE',
    'SWERVEMODULE',
    'SWERVOMETER',
    'VISION'
])

MODULE_NAMES.ROBOT = 'ROBOT'
MODULE_NAMES.SWERVEDRIVE = 'SWERVEDRIVE'
MODULE_NAMES.SWERVEMODULE = 'SWERVEMODULE'
MODULE_NAMES.SWERVOMETER = 'SWERVOMETER'
MODULE_NAMES.VISION = 'VISION'

loggingConfig = {
    MODULE_NAMES.ROBOT: False,
    MODULE_NAMES.SWERVEDRIVE: False,
    MODULE_NAMES.SWERVEMODULE: False,
    MODULE_NAMES.SWERVOMETER: False,
    MODULE_NAMES.VISION: False,    
}

dashboardConfig = {
    MODULE_NAMES.ROBOT: True,
    MODULE_NAMES.SWERVEDRIVE: True,
    MODULE_NAMES.SWERVEMODULE: True,
    MODULE_NAMES.SWERVOMETER: True,
    MODULE_NAMES.VISION: True,   
}

#all unknown values equal 0
mechanismConfig = {
    "INTAKE_BEAMBREAK_PIN": 5,
    "INTAKE_MOTOR_ID": 5,
    "INTAKE_SPEED": 1,
    "INDEX_MOTOR_ID": 61,
    "INDEX_SPEED": 0.35,
    "INDEX_ROLL_BACK_ROTATIONS": 6,
    
    "SHOOTER_LEFT_MOTOR_ID": 28,
    "SHOOTER_RIGHT_MOTOR_ID": 8,
    "SHOOTER_LEFT_SPEED": -0.85, #0.6 - good value for accuracy (on compliant wheels)
    "SHOOTER_RIGHT_SPEED": 1, #1 - good value for accuracy (on compliant wheels)
    "SHOOTER_LEFT_REVERSE_SPEED": 0.2,
    "SHOOTER_RIGHT_REVERSE_SPEED": -0.2,
    "SHOOTER_OPEN_LOOP_RAMP_RATE": 1,
    #Up and down speeds for the Sprocket Motor, NEED TO BE SET
    "SPROCKET_LEFT_MOTOR_ID": 6,
    "SPROCKET_RIGHT_MOTOR_ID": 7,
    "SPROCKET_MOTOR_LEFT_UP": 0.1,
    "SPROCKET_MOTOR_RIGHT_UP": -0.1,
    "SPROCKET_MOTOR_LEFT_DOWN": -0.1,
    "SPROCKET_MOTOR_RIGHT_DOWN": 0.1,
    #DIO channel for through bore encoder
    "SPROCKET_ENCODER_ID": 0,
    #Shift encoder values to avoid wrap around of encoder
    "SPROCKET_ENCODER_SHIFT": 20,
    #Value of sprocket encoder at 0 (in deg) (at horizontal)
    "SPROCKET_ENCODER_ZERO": 49.1, #47.9
    #Tune values for the Sprocket Motor's PID
    "SPROCKET_PID_KP": 0.03, #0.006 #0.05
    "SPROCKET_PID_KI": 0,
    "SPROCKET_PID_KD": 0, #0
    #Tune values for the Sprocket Motor's Feedforward
    "SPROCKET_FEEDFORWARD_KS": 0,
    "SPROCKET_FEEDFORWARD_KG": 0.0275, #0.0275
    "SPROCKET_FEEDFORWARD_KV": 0,
    "SPROCKET_FEEDFORWARD_KA": 0,
    "SPROCKET_FEEDFORWARD_VELOCITY": 0, #in radians
    "SPROCKET_FEEDFORWARD_ACCELERATION": 0, #in radians/second^2

    "CLIMB_MOTOR_ID": 60,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'VISION': visionConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'AUTON': autonConfig,
    'LOGGING': loggingConfig,
    'DASHBOARD': dashboardConfig
}

