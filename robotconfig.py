
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
    "INDEX_MOTOR_ID": 1,
    "INDEX_SPEED": 0,
    
    "SHOOTER_LEFT_MOTOR_ID": 61,
    "SHOOTER_RIGHT_MOTOR_ID": 62,
    "SHOOTER_LEFT_SPEED": 0.85,
    "SHOOTER_RIGHT_SPEED": -1.0,
    #Up and down speeds for the Sprocket Motor
    "SPROCKET_MOTOR_ID": 2,
    "SPROCKET_MOTOR_UP": 1,
    "SPROCKET_MOTOR_DOWN": -1,
    #Tune values for the Sprocket Motor's PID
    "SPROCKET_PID_KP": 0,
    "SPROCKET_PID_KI": 0,
    "SPROCKET_PID_KD": 0,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

