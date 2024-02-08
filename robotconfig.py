
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
    "INDEX_MOTOR_ID": 8,
    "INDEX_SPEED": 0.5,
    
    "SHOOTER_LEFT_MOTOR_ID": 28,
    "SHOOTER_RIGHT_MOTOR_ID": 61,
    "SHOOTER_LEFT_SPEED": 0.425, #0.6 - good value for accuracy (on compliant wheels)
    "SHOOTER_RIGHT_SPEED": -.50, #1 - good value for accuracy (on compliant wheels)
    #Up and down speeds for the Sprocket Motor
    "SPROCKET_MOTOR_ID": 2,
    "SPROCKET_MOTOR_UP": 1,
    "SPROCKET_MOTOR_DOWN": -1,
    #DIO channel for through bore encoder
    "SPROCKET_ENCODER_ID": 0,
    #Value of sprocket encoder at 0 (in deg)
    "SPROCKET_ENCODER_ZERO": 0,
    #Tune values for the Sprocket Motor's PID
    "SPROCKET_PID_KP": 0,
    "SPROCKET_PID_KI": 0,
    "SPROCKET_PID_KD": 0,
    #Min and max angles for sprocket motor, NEED TO BE ADJUSTED
    "SPROCKET_MOTOR_MIN_ANGLE": 30,
    "SPROCKET_MOTOR_MAX_ANGLE": 360,
    "SPROCKET_MOTOR_INTAKE_ANGLE": 30,
    "SPROCKET_MOTOR_SPEAKER_SHOOTING_ANGLE": 150,
}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

