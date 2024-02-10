
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
    #Tune values for the Sprocket Motor's Feedforward
    "SPROCKET_FEEDFORWARD_KS": 0,
    "SPROCKET_FEEDFORWARD_KG": 0,
    "SPROCKET_FEEDFORWARD_KV": 0,
    "SPROCKET_FEEDFORWARD_KA": 0,
    "SPROCKET_FEEDFORWARD_VELOCITY": 0, #in radians
    "SPROCKET_FEEDFORWARD_ACCELERATION": 0, #in radians/second^2

}

robotConfig = {
    "CONTROLLERS": controllerConfig,
    "MECHANISM": mechanismConfig,
}

