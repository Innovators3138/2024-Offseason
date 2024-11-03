import time

import wpilib
import commands2
from commands2.button import CommandXboxController

import constants

from subsystems.swerve import Swerve

from commands.drive_by_joystick_swerve import DriveByJoystickSwerve
from commands.gyro_reset import GyroReset

class RobotContainer:
    """

    """

    def __init__(self) -> None:
        self.start_time = time.time()
        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        self.drive = Swerve()
        self.configure_driver_joystick()
        self.configure_swerve_bindings()

        self.drive.setDefaultCommand(DriveByJoystickSwerve(container=self, swerve=self.drive,
                                                           field_oriented=True, rate_limited=False))

    def set_start_time(self):
        self.start_time = time.time()

    def get_enabled_time(self):
        return time.time() - self.start_time

    def configure_driver_joystick(self):
        self.driver_command_controller = CommandXboxController(constants.DRIVE_CONTROLLER_PORT)
        self.trigger_a = self.driver_command_controller.a()  # 2024 way
        self.trigger_b = self.driver_command_controller.b()
        self.trigger_x = self.driver_command_controller.x()
        self.trigger_y = self.driver_command_controller.y()
        self.trigger_rb = self.driver_command_controller.rightBumper()
        self.trigger_lb = self.driver_command_controller.leftBumper()
        self.trigger_start = self.driver_command_controller.start()
        self.trigger_back = self.driver_command_controller.back()
        self.trigger_d = self.driver_command_controller.povDown()
        self.trigger_u = self.driver_command_controller.povUp()
        self.trigger_r = self.driver_command_controller.povRight()
        self.trigger_l = self.driver_command_controller.povLeft()
        self.trigger_l_trigger = self.driver_command_controller.leftTrigger(0.2)
        self.trigger_r_trigger = self.driver_command_controller.rightTrigger(0.2)

        # BUTTONS THAT WORK ONLY WITHOUT THE LB
        self.trigger_only_a = self.trigger_a.and_(self.trigger_lb.not_())
        self.trigger_only_b = self.trigger_b.and_(self.trigger_lb.not_())
        self.trigger_only_x = self.trigger_x.and_(self.trigger_lb.not_())
        self.trigger_only_y = self.trigger_y.and_(self.trigger_lb.not_())

        # SHIFT BUTTONS THAT WORK ONLY WITH THE LB
        self.trigger_shift_a = self.trigger_a.and_(self.trigger_lb)
        self.trigger_shift_b = self.trigger_b.and_(self.trigger_lb)
        self.trigger_shift_x = self.trigger_x.and_(self.trigger_lb)
        self.trigger_shift_y = self.trigger_y.and_(self.trigger_lb)

    def configure_swerve_bindings(self):
        self.trigger_only_b.debounce(0.05).onTrue(GyroReset(self, swerve=self.drive))
        # self.trigger_y.whileTrue(DriveSwervePointTrajectory(container=self,drive=self.drive,pointlist=None,velocity=None,acceleration=None))
