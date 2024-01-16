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
    "RIGHT_SHOOTING_MOTOR_ID": 99,
    "LEFT_SHOOTING_MOTOR_ID": 99,
    "RIGHT_EJECT_SPEED": -1.0,
    "LEFT_EJECT_SPEED": 0.85,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

