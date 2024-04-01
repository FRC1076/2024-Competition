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
    'FIELD_START_POSITION': 'B', # Which of three starting positions is selected?
    'HAS_BUMPERS_ATTACHED': True, # Does the robot currently have bumpers attached?
    'USE_COM_ADJUSTMENT': False, # Should robot compensate for CoM lever arms?
    'FIELD_ORIGIN_X': 0.0, # X-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_ORIGIN_Y': 0.0, # Y-Coordinate of field orgin (center of field, viewed from scoring table)
    'FIELD_RED_A_START_POSITION_X': 308, #values of 2024 on left #248.625, #159.0, # X-Coordinate of starting position A when on red team
    'FIELD_RED_A_START_POSITION_Y': 114, #values of 2024 on left #16.75, #40.15, #54.25, # Y-Coordinate of starting postion A when on red team
    'FIELD_RED_A_START_ANGLE': 0, # Heading angle of starting position A when on red team
    'FIELD_RED_B_START_POSITION_X': 275, # X-Coordinate of starting position B when on red team
    'FIELD_RED_B_START_POSITION_Y': 57, #-28.25, # Y-Coordinate of starting postion B when on red team
    'FIELD_RED_B_START_ANGLE': 0, # Heading angle of starting position B when on red team
    'FIELD_RED_C_START_POSITION_X': 308, # X-Coordinate of starting position C when on red team
    'FIELD_RED_C_START_POSITION_Y': 0, #-137.90, # Y-Coordinate of starting postion C when on red team
    'FIELD_RED_C_START_ANGLE': 0, # Heading angle of starting position C when on red team
    'FIELD_RED_D_START_POSITION_X': 275,
    'FIELD_RED_D_START_POSITION_Y': 124,
    'FIELD_RED_D_START_ANGLE': 0.0,
    'FIELD_RED_E_START_POSITION_X': 275,
    'FIELD_RED_E_START_POSITION_Y': 0,
    'FIELD_RED_E_START_ANGLE': 0.0,
    'FIELD_BLU_A_START_POSITION_X': -308, # X-Coordinate of starting position A when on blue team
    'FIELD_BLU_A_START_POSITION_Y': 114, # 40.15, # Y-Coordinate of starting postion A when on blue team
    'FIELD_BLU_A_START_ANGLE': 180, # Heading angle of starting position A when on blue team
    'FIELD_BLU_B_START_POSITION_X': -275, # X-Coordinate of starting position B when on blue team
    'FIELD_BLU_B_START_POSITION_Y': 57, #-28.25, # Y-Coordinate of starting postion B when on blue team
    'FIELD_BLU_B_START_ANGLE': 180.0, # Heading angle of starting position B when on blue team
    'FIELD_BLU_C_START_POSITION_X': -308, # X-Coordinate of starting position C when on blue team
    'FIELD_BLU_C_START_POSITION_Y': 0, # -137.90, # Y-Coordinate of starting postion C when on blue team
    'FIELD_BLU_C_START_ANGLE': 180.0, # Heading angle of starting position C when on blue team
    'FIELD_BLU_D_START_POSITION_X': -275,
    'FIELD_BLU_D_START_POSITION_Y': 124,
    'FIELD_BLU_D_START_ANGLE': 180,
    'FIELD_BLU_E_START_POSITION_X': -275,
    'FIELD_BLU_E_START_POSITION_Y': 0,
    'FIELD_BLU_E_START_ANGLE': 180,

    'ROBOT_FRAME_DIMENSION_X': 30.0, # X-coordinate length of robot frame  (2023: 34.0)
    'ROBOT_FRAME_DIMENSION_Y': 30.0, # Y-coordinate length of robot frame  (2023: 26.0)
    'ROBOT_BUMPER_DIMENSION_X': 3.0, # Width of bumper (X-axis)
    'ROBOT_BUMPER_DIMENSION_Y': 3.0, # Width of bumper (Y-axis)
    'ROBOT_COF_OFFSET_X': 15.0, # X-offset of center of frame (assume half frame dimension) (2023: 17.0)
    'ROBOT_COF_OFFSET_Y': 15.0, # Y-offset of center of frame (assume half frame dimension) (2023: 13.0)
    'ROBOT_COM_OFFSET_X': 3, # X-offset of center of mass (relative to center of frame) #to find these we rotated the bot in place and tested
    'ROBOT_COM_OFFSET_Y': 3, # Y-offset of center of mass (relative to center of frame) #to find these we rotated the bot in place and tested
    'ROBOT_GYRO_OFFSET_X': 18.5, # X-offset of center of gyro (relative to lower left frame) 18.5 (2023: 15.0)
    'ROBOT_GYRO_OFFSET_Y': 4.25, # Y-offset of center of gyro (relative to lower left frame) 4.25 (2023: 12.0)
    'ROBOT_CAMERA_OFFSET_X': 17.0, # X-offset of center of camera lens (relative to center of frame)
    'ROBOT_CAMERA_OFFSET_Y': 0.0, # Y-offset of center of camera lens (relative to center of frame)
    'ROBOT_CAMERA_HEIGHT': 12.1875, # Height of camera eye relative to gyroscope: 11 3/16+ 2 -1
    'ROBOT_SWERVE_MODULE_OFFSET_X': 11.75, # X-offset of swerve module center from COF  (2023: 13.75)
    'ROBOT_SWERVE_MODULE_OFFSET_Y': 11.75, # Y-offset of swerve module center from COF   (2023: 9.75)
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
    'HEADING_KP': 0.007, #0.005 - reverted to this
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
    'BEARING_KP': 0.02,
    'BEARING_KI': 0.0,
    'BEARING_KD': 0.000,
    'ROBOT_INCHES_PER_ROTATION': 1.0, #1.793, # Inches per rotation of wheels
    'TELEOP_OPEN_LOOP_RAMP_RATE': 0, #0.125, # Improves maneuverability of bot.
    'TELEOP_CLOSED_LOOP_RAMP_RATE': 0,#0.125,
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
    'CAMERA_HEIGHT_FROM_GROUND': 17.3,
    'CAMERA_FORWARD_DISTANCE_FROM_COF': 12,
    'CAMERA_SIDE_DISTANCE_FROM_COF': 0.5,
    'CAMERA_PITCH': 17,
    'APRILTAGS': 0,
    'RETROREFLECTIVE': 1,
    'MIN_TARGET_ASPECT_RATIO_REFLECTIVE': 0.0,
    'MAX_TARGET_ASPECT_RATIO_REFLECTIVE': 100.0,
    'MIN_TARGET_ASPECT_RATIO_APRILTAG': 0.0,
    'MAX_TARGET_ASPECT_RATIO_APRILTAG': 100.0,
    'UPDATE_POSE': False, # True if should correct position with Limelight information. Otherwise informational.
}

#Angles of the arm at preset locations
INTAKE_ANGLE = -37 #intake angle
SHOT_ANGLE = -10 #B shot first note 
D_PRELOAD_ANGLE = -10
E_PRELOAD_ANGLE = -10
NOTE_1_ANGLE = 0 #D second note
NOTE_2_ANGLE = 0 #B shot after getting note 2
SUBWOOFER_ANGLE = -25.9 #subwoofer angle
FAR_ANGLE = 9.5 #far angle
LONG_ANGLE = 8 #long angle

#Yaw angle the robot needs to turn in place
D_YAW = 50
E_YAW = -50
NOTE_1_YAW = 25

#Wait times for specific shots
SUBWOOFER_WAIT = 0.5
SHOT_WAIT = 0.5 #seconds

autonConfig = {
    'TASK': 'TEST_NOTE',

    'B_FAST_FOUR_NOTE_SUBWOOFER': [['START_INTAKE'],
                                  ['RAISE_ARM_START', SUBWOOFER_ANGLE],
                                  ['WAIT', SUBWOOFER_WAIT],
                                  ['SHOOT_NOTE'],
                                  ['LOWER_ARM_START', INTAKE_ANGLE],
                                  ['PATH', 'B-1[FAST]'],
                                  ['RAISE_ARM_START', SHOT_ANGLE],
                                  ['PATH', '1-SHOT[FAST]'],
                                  ['SHOOT_NOTE'],
                                  ['LOWER_ARM_START', INTAKE_ANGLE],
                                  ['PATH', "SHOT-2"],
                                  ['RAISE_ARM_START', NOTE_2_ANGLE],
                                  ['WAIT', 0.5],
                                  ['SHOOT_NOTE'],
                                  ['LOWER_ARM_START', INTAKE_ANGLE],
                                  ['PATH', '2-3[FAST]'],
                                  ['RAISE_ARM_START', SUBWOOFER_ANGLE],
                                  ['PATH', '3-B'],
                                  ['SHOOT_NOTE'],
                                  ['LOWER_ARM_START', INTAKE_ANGLE]],


    'B_FAR_FOUR_NOTE_TEST': [['START_INTAKE'],
                            ['RAISE_ARM_START', SUBWOOFER_ANGLE],
                            ['WAIT', SUBWOOFER_WAIT],
                            ['SHOOT_NOTE'],
                            ['LOWER_ARM_START', INTAKE_ANGLE],
                            ['PATH', 'B-2'],
                            ['RAISE_ARM_START', NOTE_2_ANGLE],
                            ['WAIT', SHOT_WAIT],
                            ['SHOOT_NOTE'],
                            ['LOWER_ARM_START', INTAKE_ANGLE],
                            ['PATH', '2-4'],
                            ['RAISE_ARM_START', FAR_ANGLE],
                            ['PATH', '4-FAR'],
                            ['SHOOT_NOTE'],
                            ['LOWER_ARM_START', INTAKE_ANGLE],
                            ['PATH', 'FAR-5'],
                            ['RAISE_ARM_START', FAR_ANGLE],
                            ['PATH', '5-FAR'],
                            ['SHOOT_NOTE'],
                            ['LOWER_ARM_START', INTAKE_ANGLE]],

    'B_CENTER_FOUR_NOTE': [['START_INTAKE'],
                          ['RAISE_ARM_START', SUBWOOFER_ANGLE],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'B-2'],
                          ['RAISE_ARM_START', NOTE_2_ANGLE],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', '2-6'],
                          ['RAISE_ARM_START', FAR_ANGLE],
                          ['PATH', '6-FAR'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'FAR-5'],
                          ['RAISE_ARM_START', FAR_ANGLE],
                          ['PATH', '5-FAR'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE]],

    'D_CENTER_FOUR_NOTE': [['START_INTAKE'],
                          ['RAISE_ARM_START', D_PRELOAD_ANGLE],
                          ['ROTATE', D_YAW],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'D-1'],
                          ['ROTATE', NOTE_1_YAW],
                          ['RAISE_ARM_START', NOTE_1_ANGLE],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', '1-4'],
                          ['RAISE_ARM_START', FAR_ANGLE],
                          ['PATH', '4-FAR'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'FAR-6'],
                          ['RAISE_ARM_START', FAR_ANGLE],
                          ['PATH', '6-FAR'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE]],

    'D_CENTER5_FOUR_NOTE': [['START_INTAKE'],
                          ['RAISE_ARM_START', D_PRELOAD_ANGLE],
                          ['ROTATE', D_YAW],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'D-1'],
                          ['ROTATE', NOTE_1_YAW],
                          ['RAISE_ARM_START', NOTE_1_ANGLE],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', '1-5'],
                          ['RAISE_ARM_START', FAR_ANGLE],
                          ['PATH', '5-FAR'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'FAR-6'],
                          ['RAISE_ARM_START', FAR_ANGLE],
                          ['PATH', '6-FAR'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE]],

    #Incomplete
    """
    'E_THREE_NOTE':       [['START_INTAKE'],
                          ['RAISE_ARM_START', E_PRELOAD_ANGLE],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'E-8'],
                          ['RAISE_ARM_START', LONG_ANGLE],
                          ['PATH', '8-LONG'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'LONG-7'],
                          ['RAISE_ARM_START', LONG_ANGLE],
                          ['PATH', '7-LONG'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE]],
    """

   'E_CENTER_THREE_NOTE': [['START_INTAKE'],
                          ['ROTATE', E_YAW],
                          ['RAISE_ARM_START', E_PRELOAD_ANGLE],
                          ['WAIT', SHOT_WAIT],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'E-7'],
                          ['RAISE_ARM_START', LONG_ANGLE],
                          ['PATH', '7-LONG'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE],
                          ['PATH', 'LONG-8'],
                          ['RAISE_ARM_START', LONG_ANGLE],
                          ['PATH', '8-LONG'],
                          ['SHOOT_NOTE'],
                          ['LOWER_ARM_START', INTAKE_ANGLE]],

    'TEST_NOTE': [['PATH', 'TEST']],

    #'B_FOUR_NOTE_GRAB': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', 'B-1', 0.5], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', SHOT_2, 0.3], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', '2-3', 0.3], ['RAISE_ARM_START', 0], ['PATH', '3-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'NOTE_GRAB_TEST': [['START_INTAKE'], ['MOVE_TO_NOTE', -(275 - 64), 57, 180, 0], ['RAISE_ARM_START', 0]],
    #'B_FAST_FOUR_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1[FAST]'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT[FAST]'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3[FAST]'], ['RAISE_ARM_START', 0], ['PATH', '3-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'B_FOUR_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3'], ['RAISE_ARM_START', 0], ['PATH', '3-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'B_THREE_NOTE_STAGE_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3'], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['PATH', '3-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'B_THREE_NOTE_AMP_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-4']],
    #'B_ONE_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-8']],
    #'A_ONE_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -7], ['PATH', 'A-AOUT'], ['ROTATE', 75], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['WAIT', 7], ['ROTATE', 0], ['PATH', 'A-LEAVE']],
    #'C_ONE_NOTE':[['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'C-8-ROTATE']],

    #'A_THREE_NOTE_AMP_SIDE_CENTER_LINE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'A-5']],
    #A_THREE_NOTE_AMP_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'A-1'], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '1-SHOT'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'C_TWO_NOTE_CENTER_LINE':[['START_INTAKE'], ['RAISE_ARM_START', -7], ['PATH', 'C-COUT'], ['ROTATE', -75], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['ROTATE', 0], ['PATH', 'COUT-8'], ['RAISE_ARM_START', -7], ['PATH', '8-COUT'], ['ROTATE', -75], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'C_TWO_NOTE_CENTER_LINE_NOTE_GRAB':[['START_INTAKE'], ['RAISE_ARM_START', -7], ['PATH', 'C-COUT'], ['ROTATE', -75], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['ROTATE', 0], ['PATH_TO_NOTE', 'COUT-8', 1.5], ['RAISE_ARM_START', -7], ['PATH', '8-COUT'], ['ROTATE', -75], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'A_THREE_NOTE_NO_CENTER': [['START_INTAKE'], ['RAISE_ARM_START', -15], ['PATH', 'A-ABACK'], ['ROTATE', 50], ['SHOOT_NOTE'], ['ROTATE', 0], ['WAIT', 1.5], ['LOWER_ARM_START', -37], ['PATH', 'ABACK-1'], ['RAISE_ARM_START', 0], ['PATH', '1-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3'], ['RAISE_ARM_START', -25.9], ['PATH', '3-B'], ['SHOOT_NOTE']],
    #'B_FOUR_NOTE_GRAB_AMP_SIDE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1[FAST]'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT[FAST]'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', '2-4', 1.5], ['RAISE_ARM_START', -25.9], ['PATH', '4-B'], ['SHOOT_NOTE']],
    #'B_THREE_HALF_NOTE_GRAB_STAGE_SIDE_DRAGONS': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3'], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['PATH', '3-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', '2-6', 1.5], ['PATH', '6-STAGE']],
    #'B_THREE_HALF_NOTE_GRAB_AMP_SIDE_DRAGONS': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1[FAST]'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT[FAST]'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', '2-6', 1.5], ['PATH', '6-STAGE']],
    #'B_FIVE_NOTE_TEST': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1[FAST]'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT[FAST]'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-4'], ['RAISE_ARM_START', 0], ['PATH', '4-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3[FAST]'], ['RAISE_ARM_START', -23], ['PATH', '3-B'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],
    #'B_FAR_FIVE_NOTE_TEST': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1[FAST]'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT[FAST]'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-4'], ['RAISE_ARM_START', 9.5], ['PATH', '4-FAR'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'FAR-5'], ['RAISE_ARM_START', 9.5], ['PATH', '5-FAR'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],

    #'B_FAST_FOUR_NOTE': [['START_INTAKE'], ['RAISE_ARM_START', -25.9], ['WAIT', 1], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-1[FAST]'], ['RAISE_ARM_START', -10], ['PATH', '1-SHOT[FAST]'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', SHOT_2], ['RAISE_ARM_START', 0], ['WAIT', 0.5], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', '2-3[FAST]'], ['RAISE_ARM_START', -23], ['PATH', '3-B'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],

    #'B_FOUR_NOTE_TWO_FAR_GRAB': [['START_INTAKE'], ['WAIT', 0.5], ['RAISE_ARM_START', -25.9], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH', 'B-2'], ['RAISE_ARM_START', 0], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37], ['PATH_TO_NOTE', '2-5', 1.5], ['RAISE_ARM_START', 0], ['PATH', '5-2'], ['SHOOT_NOTE'], ['LOWER_ARM_START', -37]],

    #'NOTE_GRAB_TEST': [['START_INTAKE'], [[]'POINT_TO_NOTE', True, ['ROTATE', -90]]],


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
    ### BLUE TEAM ###
    #     'NOTE 9': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -207.03, 57, 180]],
    #     'NOTE 10': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -216.875, 114, 180], ['MOVE', -207.03, 114, 180]],
    #     'NOTE 11': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -216.875, 0, 180], ['MOVE', -207.03, 0, 180]],
    #     'NOTE 12': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, -75, 180], ['MOVE', 19.9, -75, 180]],
    #     'NOTE 13': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, -34.31, 180], ['MOVE', 19.9, -9, 180]],
    #     'NOTE 14 NEGATIVE': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, -34.41, 180], ['MOVE', 19.9, 57, 180]],
    #     'NOTE 14 POSITIVE': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, 148.31, 180], ['MOVE', 19.9, 57, 180]],
    #     'NOTE 15': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, 148.31, 180], ['MOVE', 19.9, 123, 180]],
    #     'NOTE 16': [['WAIT', 2], ['UPDATE_POSE'], ['MOVE', -91.84, 189, 180], ['MOVE', 19.9, 189, 180]],

    'AUTON_CLOSED_LOOP_RAMP_RATE': 0,
    'AUTON_OPEN_LOOP_RAMP_RATE': 0, # Improves the quality of swervometery by avoiding slippage.
    'BALANCE_BOT': True,
    'DO_COMMUNITY': False, # Only applies for position B
    'DRIVE_BASE_RADIUS': 0.5388,
    'MAX_SPEED_M/S': 4.3,
    'PERIOD': 0.02,
    'ROTATION_KD': 0,
    'ROTATION_KI': 0,
    'ROTATION_KP': 5,
    'SCORE_EXISTING': True,
    'TRANSLATION_KD': 0,
    'TRANSLATION_KI': 0,
    'TRANSLATION_KP': 3,
    'MAX_PICK_UP_DISTANCE': 70,
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
    "PDH_LOGGING": False,
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

noteDetectorConfig = {
    "CAMERA_HEIGHT": 15.75,
    "NOTE_HEIGHT": 2.0,
    "NOTE_DIAMETER": 14.0,
    "CAMERA_ANGLE_ABOVE_HORIZONTAL": -37.5,
    "CAMERA_OFFSET_X": 0,
    "CAMERA_OFFSET_Y": 10.5,
    "CAMERA_FOV_Z": 41.232,
    "CAMERA_FOV_X": 62.548,
    "CAMERA_PIXELS_Z": 300,
    "CAMERA_PIXELS_X": 300,
    "NETWORKTABLES_IP": '10.10.76.2',
    "DEFAULT_BBOX": -1000,
    "INTAKE_RIGHT_ERROR_MARGIN": 6,
    "INTAKE_LEFT_ERROR_MARGIN": -6,
}

elasticConfig = {
    "AUTONLIST": 0,
    "NOTE_IS_DETECTED": False,
    "NOTE_ON_LEFT": False,
    "NOTE_ON_RIGHT": False,
    "NOTE_IS_LOADED": False,

}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
    "NOTEDETECTOR": noteDetectorConfig,
    'SWERVOMETER': swervometerConfig, # Must be BEFORE drivetrain
    'VISION': visionConfig, # Must be BEFORE drivetrain
    'DRIVETRAIN': drivetrainConfig,
    'AUTON': autonConfig,
    'LOGGING': loggingConfig,
    'DASHBOARD': dashboardConfig,
}

