import math
import typing

import commands2
import wpilib

from commands2 import Subsystem
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d, Translation3d, Pose3d, Rotation3d
from wpimath.kinematics import ChassisSpeeds, SwerveModuleState, SwerveDrive4Kinematics, SwerveDrive4Odometry
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.controller import PIDController

import navx
import rev
import phoenix5
import pathplannerlib
from pathplannerlib.path import PathPlannerTrajectory
import robotpy_apriltag

import constants
from swervemodule import SwerveModule

class Swerve(Subsystem):
    def __init__(self):
        super().__init__()

        self.front_left = SwerveModule(
            driving_can_id=constants.LEFT_FRONT_DRIVE_CAN_ID,
            turning_can_id=constants.LEFT_FRONT_STEER_CAN_ID,
            turning_encoder_offset=constants.LF_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='fl',
        )
        self.front_right = SwerveModule(
            driving_can_id=constants.RIGHT_FRONT_DRIVE_CAN_ID,
            turning_can_id=constants.RIGHT_FRONT_STEER_CAN_ID,
            turning_encoder_offset=constants.RF_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='fr',
        )
        self.back_left = SwerveModule(
            driving_can_id=constants.LEFT_BACK_DRIVE_CAN_ID,
            turning_can_id=constants.LEFT_BACK_STEER_CAN_ID,
            turning_encoder_offset=constants.LB_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='bl',
        )
        self.back_right = SwerveModule(
            driving_can_id=constants.RIGHT_BACK_DRIVE_CAN_ID,
            turning_can_id=constants.RIGHT_BACK_STEER_CAN_ID,
            turning_encoder_offset=constants.RB_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='br',
        )

        self.swerve_modules = [self.front_left, self.front_right, self.back_left, self.back_right]

        self.navx = navx.AHRS.create_spi(update_rate_hz=50)
        if self.navx.isCalibrating():
            print('Unable to reset navx: Calibration in progress')
        else:
            pass

        self.navx.zeroYaw()
        self.gyro_calibrated = False


