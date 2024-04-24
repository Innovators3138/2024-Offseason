from commands2 import Subsystem

from wpilib import (
    RobotBase,
    SmartDashboard,
    Timer,
    DataLogManager,
    DriverStation,
)

from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)

from wpimath.estimator import SwerveDrive4PoseEstimator

import constants

from util import convenientmath
from util.angleoptimize import optimize_angle

class SwerveModuleConfigParams:
    swerve_encoder_offset: float
    swerve_encoder_inverted: bool
    drive_encoder_inverted: bool
    drive_motor_id: int
    drive_motor_inverted: bool
    steer_motor_id: int
    steer_motor_inverted: bool

    def __init__(
        self,
        drive_motor_id: int,
        drive_motor_inverted: bool,
        steer_motor_id: int,
        steer_motor_inverted: bool,
        swerve_encoder_offset: float,
        drive_encoder_inverted: bool,
        swerve_encoder_inverted: bool,
    ) -> None:
        self.drive_motor_id = drive_motor_id
        self.drive_motor_inverted = drive_motor_inverted
        self.steer_motor_id = steer_motor_id
        self.steer_motor_inverted = steer_motor_inverted
        self.swerve_encoder_offset = swerve_encoder_offset
        self.drive_encoder_inverted = drive_encoder_inverted
        self.swerve_encoder_inverted = swerve_encoder_inverted

class SwerveModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def get_swerve_angle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def set_swerve_angle(self, swerve_angle: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")
