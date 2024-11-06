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
from subsystems.swervemodule import SwerveModule

class Swerve(Subsystem):
    def __init__(self):
        super().__init__()

        self.counter = 6

        self.front_left = SwerveModule(
            driving_can_id=constants.FRONT_LEFT_DRIVE_CAN_ID,
            steering_can_id=constants.FRONT_LEFT_STEER_CAN_ID,
            steering_encoder_offset=constants.FL_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='fl',
        )
        self.front_right = SwerveModule(
            driving_can_id=constants.FRONT_RIGHT_DRIVE_CAN_ID,
            steering_can_id=constants.FRONT_RIGHT_STEER_CAN_ID,
            steering_encoder_offset=constants.FR_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='fr',
        )
        self.back_left = SwerveModule(
            driving_can_id=constants.BACK_LEFT_DRIVE_CAN_ID,
            steering_can_id=constants.BACK_LEFT_STEER_CAN_ID,
            steering_encoder_offset=constants.BL_ZERO_OFFSET,
            driving_inverted=constants.DRIVE_MOTORS_INVERTED,
            turning_inverted=constants.TURN_MOTORS_INVERTED,
            label='bl',
        )
        self.back_right = SwerveModule(
            driving_can_id=constants.BACK_RIGHT_DRIVE_CAN_ID,
            steering_can_id=constants.BACK_RIGHT_STEER_CAN_ID,
            steering_encoder_offset=constants.BR_ZERO_OFFSET,
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

        self.current_rotation, self.current_translation_dir, self.current_translation_mag = 0.0, 0.0, 0.0
        self.fwd_mag_limiter = SlewRateLimiter(0.9 * constants.MAGNITUDE_SLEW_RATE)
        self.strafe_mag_limiter = SlewRateLimiter(constants.MAGNITUDE_SLEW_RATE)
        self.rot_limiter = SlewRateLimiter(constants.ROTATIONAL_SLEW_RATE)


        self.pose_estimator = SwerveDrive4PoseEstimator(constants.DRIVE_KINEMATICS,
                                                        Rotation2d.fromDegrees(self.get_angle()),
                                                        self.get_module_positions(),
                                                        initialPose=Pose2d(constants.START_X, constants.START_Y, Rotation2d.fromDegrees(self.get_angle())))

    def get_pose(self) -> Pose2d:
        return self.pose_estimator.getEstimatedPosition()

    def reset_odometry(self, pose: Pose2d) -> None:
        """Resets the odometry to the specified pose.
        """
        self.pose_estimator.resetPosition(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(), pose)

    def drive(self, x_speed: float, y_speed: float, rot: float, field_relative: bool, rate_limited: bool) -> None:
        if rate_limited:
            x_speed_commanded = self.fwd_mag_limiter.calculate(x_speed)
            y_speed_commanded = self.strafe_mag_limiter.calculate(y_speed)
            rotation_commanded = self.rot_limiter.calculate(rot)
        else:
            x_speed_commanded = x_speed
            y_speed_commanded = y_speed
            rotation_commanded = rot


        # Convert the commanded speeds into the correct units for the drivetrain
        x_speed_delivered = x_speed_commanded * constants.MAX_SPEED_METERS_PER_SECOND
        y_speed_delivered = y_speed_commanded * constants.MAX_SPEED_METERS_PER_SECOND
        rot_delivered = rotation_commanded * constants.MAX_ANGULAR_SPEED

        if constants.SWERVE_STATE_MESSAGES:
            wpilib.SmartDashboard.putNumberArray('_xyr', [x_speed_delivered, y_speed_delivered, rot_delivered])

        # Create the swerve state array depending on if we are field relative or not
        swerve_module_states = constants.DRIVE_KINEMATICS.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered, Rotation2d.fromDegrees(self.get_angle()),)
            if field_relative else ChassisSpeeds(x_speed_delivered, y_speed_delivered, rot_delivered))

        # normalize wheel speeds so we do not exceed our speed limit
        swerve_module_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(swerve_module_states, constants.MAX_TOTAL_SPEED)
        for state, module in zip(swerve_module_states, self.swerve_modules):
            module.setDesiredState(state)

    def set_module_states(self, desired_states: typing.Tuple[SwerveModuleState, SwerveModuleState, SwerveModuleState, SwerveModuleState]) -> None:
        desired_states = SwerveDrive4Kinematics.desaturateWheelSpeeds(desired_states, constants.MAX_TOTAL_SPEED)
        for idx, m in enumerate(self.swerve_modules):
            m.setDesiredState(desired_states[idx])

    def reset_encoders(self) -> None:
        [m.resetEncoders() for m in self.swerve_modules]

    def zero_heading(self) -> None:
        self.navx.reset()

    def get_heading(self) -> float:
        return Rotation2d.fromDegrees(self.get_angle()).degrees()

    def get_turn_rate(self) -> float:
        return self.navx.getRate() * (-1.0 if constants.GYRO_REVERSED else 1.0)

    def get_module_positions(self):
        return [m.getPosition() for m in self.swerve_modules]

    def get_module_states(self):
        return [m.getState() for m in self.swerve_modules]

    def get_raw_angle(self):
        return self.navx.getAngle()

    def get_angle(self):
        return -self.navx.getAngle() if constants.GYRO_REVERSED else self.navx.getAngle()

    def get_yaw(self):
        return -self.navx.getYaw() if constants.GYRO_REVERSED else self.navx.getYaw()

    def get_pitch(self):
        pitch_offset = 0
        return self.navx.getPitch() - pitch_offset

    def get_roll(self):
        roll_offset = 0
        return self.navx.getRoll() - roll_offset

    def reset_gyro(self, adjustment=None):
        self.navx.reset()
        if adjustment is not None:
            self.navx.setAngleAdjustment(adjustment)

    def periodic(self) -> None:
        self.counter += 1
        wpilib.SmartDashboard.putNumber('_timestamp', wpilib.Timer.getFPGATimestamp())

        for module in self.swerve_modules:
            wpilib.SmartDashboard.putNumber(f"{module.label} steering motor position:", module.get_steer_encoder())

        if wpilib.RobotBase.isReal():
            self.pose_estimator.updateWithTime(wpilib.Timer.getFPGATimestamp(), Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions(),)
        else:
            self.pose_estimator.update(Rotation2d.fromDegrees(self.get_angle()), self.get_module_positions())
            pose = self.get_pose()
            wpilib.SmartDashboard.putNumberArray('real_robot_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])

        if self.counter % 10 == 0:
            pose = self.get_pose()
            if wpilib.RobotBase.isReal():
                wpilib.SmartDashboard.putNumberArray('drive_pose', [pose.X(), pose.Y(), pose.rotation().degrees()])
                wpilib.SmartDashboard.putNumber('drive_x', pose.X())
                wpilib.SmartDashboard.putNumber('drive_y', pose.Y())
                wpilib.SmartDashboard.putNumber('drive_theta', pose.rotation().degrees())

            ypr = [self.get_yaw(), self.get_pitch(), self.get_roll(), self.navx.getRotation2d().degrees()]
            wpilib.SmartDashboard.putNumberArray('_navx_YPR', ypr)

