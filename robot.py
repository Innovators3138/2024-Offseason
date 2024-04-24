import wpilib
import commands2

from robotcontainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    """

    """

    def __init__(self):
        super().__init__()
        self.container = None

    def robotInit(self) -> None:
        """

        :return:
        """
        self.container = RobotContainer()

    def teleopInit(self) -> None:
        pass

    def teleopPeriodic(self) -> None:
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
