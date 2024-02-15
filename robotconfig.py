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
    
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
    "NOTEDETECTOR": noteDetectorConfig,
}

