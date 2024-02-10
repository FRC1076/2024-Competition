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
    "LEFT_SHOOTING_MOTOR_ID": 8,
    "RIGHT_SHOOTING_MOTOR_ID": 28,
    "RIGHT_EJECT_SPEED": 1.0,
    "LEFT_EJECT_SPEED": -0.6,

    "INDEX_MOTOR_ID": 17,
    "INDEX_MOTOR_UP_SPEED": -0.5,
    "INDEX_MOTOR_DOWN_SPEED": 0.5,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

