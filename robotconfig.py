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

#all unknown values equal 0
mechanismConfig = {
    "INTAKE_BEAMBREAK_PIN": 0,
    "INTAKE_MOTOR_ID": 0,
    "INTAKE_SPEED": 0,
    "TRANSPORT_MOTOR_ID": 0,
    "TRANSPORT_SPEED": 0,
    "SHOOTER_LEFT_MOTOR_ID": 61,
    "SHOOTER_RIGHT_MOTOR_ID": 62,
    "SHOOTER_LEFT_SPEED": 0.85,
    "SHOOTER_RIGHT_SPEED": -1.0,
    "HOOD_MOTOR_ID": 0,
    "HOOD_SPEED": 0,
    
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

