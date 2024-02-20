DEADZONE = 0.1

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

mechanismConfig = {
    "LEFT_SHOOTING_MOTOR_ID": 61,
    "RIGHT_SHOOTING_MOTOR_ID": 62,
    "RIGHT_EJECT_SPEED": -1.0,
    "LEFT_EJECT_SPEED": 0.85,
}

noteDetectorConfig = {
    "CAMERA_HEIGHT": 11.9,
    "NOTE_HEIGHT": 2.0,
    "CAMERA_ANGLE_ABOVE_HORIZONTAL": 0,
    "CAMERA_OFFSET_X": 0,
    "CAMERA_OFFSET_Y": 0,
    "CAMERA_FOV_Z": 41.232,
    "CAMERA_FOV_X": 62.548, 
    "CAMERA_PIXELS_Z": 300,
    "CAMERA_PIXELS_X": 300,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
    "NOTEDETECTOR": noteDetectorConfig,
}

