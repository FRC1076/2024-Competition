from collections import namedtuple
import wpilib

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
    "RIGHT_SHOOTING_MOTOR_ID": 1,
    "LEFT_SHOOTING_MOTOR_ID": 2,
    "RIGHT_EJECT_SPEED": 0.1,
    "LEFT_EJECT_SPEED": -0.1,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig
}

