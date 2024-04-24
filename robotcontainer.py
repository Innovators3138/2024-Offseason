import wpilib
import commands2
import commands2.button

import constants


class RobotContainer:
    """

    """

    def __init__(self) -> None:
        wpilib.DataLogManager.start()
        wpilib.DataLogManager.logNetworkTables(True)
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)

    def config_button_bindings(self):
        pass
