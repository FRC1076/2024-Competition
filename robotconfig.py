
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
    "INTAKE_BEAMBREAK_PIN": 1,
    "INTAKE_MOTOR_ID": 5,
    "INTAKE_SPEED": 0,
    "INDEX_MOTOR_ID": 8,
    "INDEX_SPEED": 0.5,
    
    "SHOOTER_LEFT_MOTOR_ID": 28,
    "SHOOTER_RIGHT_MOTOR_ID": 61,
    "SHOOTER_LEFT_SPEED": 0.425, #0.6 - good value for accuracy (on compliant wheels)
    "SHOOTER_RIGHT_SPEED": -.50, #1 - good value for accuracy (on compliant wheels)
    #Up and down speeds for the Sprocket Motor, NEED TO BE SET
    "SPROCKET_LEFT_MOTOR_ID": 6,
    "SPROCKET_RIGHT_MOTOR_ID": 7,
    "SPROCKET_MOTOR_LEFT_UP": 0.5,
    "SPROCKET_MOTOR_RIGHT_UP": -0.5,
    "SPROCKET_MOTOR_LEFT_DOWN": -0.5,
    "SPROCKET_MOTOR_RIGHT_DOWN": 0.5,
    #DIO channel for through bore encoder
    "SPROCKET_ENCODER_ID": 0,
    #Value of sprocket encoder at 0 (in deg)
    "SPROCKET_ENCODER_ZERO": 0,
    #Tune values for the Sprocket Motor's PID
    "SPROCKET_PID_KP": 0,
    "SPROCKET_PID_KI": 0,
    "SPROCKET_PID_KD": 0,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

