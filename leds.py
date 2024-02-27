import wpilib

class LEDs:  
    def rainbowLED(color):
        # Pin 7 is the first value, Pin 8 is the second value, Pin 9 is the third value in the binary number
        # First = 1s place, Second = 2s place, Third = 4s place
        # 1 = red, 2 = orange, 3 = yellow, 4 = green, 5 = blue, 6 = purple, 0 = off
        # true is 1, false is 0
        digitalPin7 = wpilib.DigitalOutput(7)
        digitalPin8 = wpilib.DigitalOutput(8)
        digitalPin9 = wpilib.DigitalOutput(9)

        if color == "red":
            digitalPin7.set(True)
            digitalPin8.set(False)
            digitalPin9.set(False)
        elif color == "orange":
            digitalPin7.set(False)
            digitalPin8.set(True)
            digitalPin9.set(False)  
        elif color == "yellow":
            digitalPin7.set(True)
            digitalPin8.set(True)
            digitalPin9.set(False)
        elif color == "green":
            digitalPin7.set(False)
            digitalPin8.set(False)
            digitalPin9.set(True)
        elif color == "blue":
            digitalPin7.set(True)
            digitalPin8.set(False)
            digitalPin9.set(True)
        elif color == "purple":
            digitalPin7.set(False)
            digitalPin8.set(True)
            digitalPin9.set(True)
        
        return