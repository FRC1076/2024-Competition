from wpilib.shuffleboard import Shuffleboard
from wpilib.shuffleboard import BuiltInWidgets

class Elastic:

    def __init__(self):
        self.placeholder = self

    def displayMainWindow(self):
        tab = "Main Window"
        (Shuffleboard.getTab(tab)
        .add("Note Detected!", 1)
        .withSize(4,4)
        .withPosition(17,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())

        (Shuffleboard.getTab(tab)
        .add("Note Loaded!", 1) #check with beambreak
        .withSize(4,4)
        .withPosition(21,0)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .getEntry())
        