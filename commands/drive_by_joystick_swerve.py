import math
import typing
import commands2
import wpilib

from subsystems.swerve import Swerve
from wpilib import SmartDashboard
from commands2.button import CommandXboxController
from wpimath.geometry import Translation2d
from wpimath.filter import Debouncer
import constants

class DriveByJoystickSwerve(commands2.Command):
    def __init__(self, container, swerve: Swerve, field_oriented=True, rate_limited=False,) -> None:
        super().__init__()
        self.setName('drive_by_joystick_swerve')
        self.container = container
        self.swerve = swerve
        self.field_oriented = field_oriented
        self.rate_limited = rate_limited
        self.addRequirements(*[self.swerve])

        self.controller: typing.Optional[CommandXboxController] = self.container.driver_command_controller
        self.robot_oriented_trigger = self.controller.leftBumper()
        self.debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)
        self.robot_oriented_debouncer = Debouncer(0.1, Debouncer.DebounceType.kBoth)

    def initialize(self) -> None:
        """Called just before this Command runs the first time."""
        self.start_time = round(self.container.get_enabled_time(), 2)
        print("\n" + f"** Started {self.getName()} at {self.start_time} s **", flush=True)
        SmartDashboard.putString("alert", f"** Started {self.getName()} at {self.start_time - self.container.get_enabled_time():.1f} s **")
        self.slowmode_history = [1 for i in range(20)]

    def execute(self) -> None:
        slowmode_multiplier = 0.2 + 0.8 * self.controller.getRightTriggerAxis()
        angular_slowmode_multiplier = 0.5 + 0.5 * self.controller.getRightTriggerAxis()

        if self.robot_oriented_debouncer.calculate(self.robot_oriented_trigger.getAsBoolean()):
            self.field_oriented = False
        else:
            self.field_oriented = True

        max_linear = 1 * slowmode_multiplier
        max_angular = 1 * angular_slowmode_multiplier

        joystick_fwd = self.controller.getLeftY()
        joystick_strafe = self.controller.getLeftX()

        linear_mapping = True
        if linear_mapping:
            desired_fwd = -self.input_transform_linear(1.0 * joystick_fwd) * max_linear
            desired_strafe = -self.input_transform_linear(1.0 * joystick_strafe) * max_linear
            desired_rot = -self.input_transform_linear(1.0 * self.controller.getRightX()) * max_angular
        else:
            angle = math.atan2(joystick_fwd, joystick_strafe)
            correction = math.fabs(math.cos(angle) * (math.sin(angle)))  # peaks at 45 degrees
            fwd = joystick_fwd * (1 + 0.3 * math.copysign(correction, joystick_fwd))
            strafe = joystick_strafe * (1 + 0.3 * math.copysign(correction, joystick_strafe))
            if wpilib.RobotBase.isSimulation():
                wpilib.SmartDashboard.putNumberArray('joysticks', [joystick_fwd, joystick_strafe, correction, fwd, strafe])

            desired_fwd = -self.input_transform(1.0 * fwd) * max_linear
            desired_strafe = -self.input_transform(1.0 * strafe) * max_linear
            desired_rot = -self.input_transform(1.0 * self.controller.getRightX()) * max_angular

        if wpilib.RobotBase.isSimulation():
            SmartDashboard.putNumberArray('joystick', [desired_fwd, desired_strafe, desired_rot])

        self.swerve.drive(x_speed=desired_fwd, y_speed=desired_strafe, rot=desired_rot,
                          field_relative=self.field_oriented, rate_limited=self.rate_limited)

    def end(self, interrupted: bool) -> None:
        self.swerve.drive(0, 0, 0, field_relative=self.field_oriented, rate_limited=False)
        end_time = self.container.get_enabled_time()
        message = 'Interrupted' if interrupted else 'Ended'
        print(f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s")
        SmartDashboard.putstring(f"alert", f"** {message} {self.getName()} at {end_time:.1f} s after {end_time - self.start_time:.1f} s")

    def apply_deadband(self, value, db_low=constants.INNER_DEADBAND, db_high=constants.OUTER_DEADBAND):
        if abs(value) < db_low:
            return 0
        elif abs(value) > db_high:
            return 1 * math.copysign(1, value)
        else:
            return value

    def input_transform(self, value, a=0.9, b=0.1):
        db_value = self.apply_deadband(value)
        return a * db_value**3 + b * db_value

    def input_transform_linear(self, value, a=0.9, b=0.1):
        db_value = self.apply_deadband(value)
        return db_value