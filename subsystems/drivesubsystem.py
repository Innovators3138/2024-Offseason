import time
from enum import Enum, auto
from typing import Tuple
import typing

import phoenix5
import rev
from navx import AHRS
from wpilib import (
    DataLogManager,
    RobotBase,
    SmartDashboard,
    SPI,
    Timer,
)
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.filter import SlewRateLimiter
from wpimath.kinematics import (
    ChassisSpeeds,
    SwerveModuleState,
    SwerveDrive4Kinematics,
    SwerveDrive4Odometry,
    SwerveModulePosition,
)
from wpimath.units import degreesToRadians

from wpimath.estimator import SwerveDrive4PoseEstimator

from commands2 import Subsystem

from phoenix5 import WPI_TalonSRX, FeedbackDevice
from util.LazyCansparkMax import LazyCANSparkMax, SparkMaxPIDController

from util.angleoptimize import optimize_angle
import constants


class SwerveModuleConfigParams:
    drive_motor_id: int
    drive_motor_inverted: bool
    steer_motor_id: int
    steer_motor_inverted: bool
    swerve_encoder_offset: float
    sensor_phase_inverted: bool

    def __init__(
            self,
            drive_motor_id: int,
            drive_motor_inverted: bool,
            steer_motor_id: int,
            steer_motor_inverted: bool,
            swerve_encoder_offset: float,
            sensor_phase_inverted: bool) -> None:
        self.drive_motor_id = drive_motor_id
        self.drive_motor_inverted = drive_motor_inverted
        self.steer_motor_id = steer_motor_id
        self.steer_motor_inverted = steer_motor_inverted
        self.swerve_encoder_offset = swerve_encoder_offset
        self.sensor_phase_inverted = sensor_phase_inverted

class SwerveModule:
    def __init__(self, name: str) -> None:
        self.name = name

    def get_swerve_angle(self) -> Rotation2d:
        raise NotImplementedError("Must be implemented by subclass")

    def set_swerve_angle(self, swerve_angle: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def set_swerve_angle_target(self, swerve_angle_target: Rotation2d) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def get_wheel_linear_velocity(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def get_wheel_total_position(self) -> float:
        raise NotImplementedError("Must be implemented by subclass")

    def set_wheel_linear_velocity_target(self, wheel_linear_velocity_target: float) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def reset(self) -> None:
        raise NotImplementedError("Must be implemented by subclass")

    def optimized_angle(self, target_angle: Rotation2d) -> Rotation2d:
        return optimize_angle(self.get_swerve_angle(), target_angle)

    def get_position(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.get_wheel_total_position(), self.get_swerve_angle())

    def get_state(self) -> SwerveModuleState:
        return SwerveModuleState(
            self.get_wheel_linear_velocity(),
            self.get_swerve_angle(),
        )

    def apply_state(self, state: SwerveModuleState) -> None:
        optimized_state = SwerveModuleState.optimize(state, self.get_swerve_angle())

        self.set_wheel_linear_velocity_target(optimized_state.speed)
        if (
            abs(optimized_state.speed) >= constants.MIN_WHEEL_LINEAR_VELOCITY
        ):
            optimized_angle = self.optimized_angle(optimized_state.angle)
            self.set_swerve_angle_target(optimized_angle)


class WCPSwerveModule(SwerveModule):
    """
    Implementation of West Coast Products Swerve Module
    drive_motor: NEO with built in encoder attached to wheel with gearing
    steer_motor: MiniCIM with CTRE Mag Encoder attached to swerve with belt

    """
    def __init__(self, name: str, config: SwerveModuleConfigParams):
        SwerveModule.__init__(self, name)
        DataLogManager.log(f"Initializing swerve module: {self.name}")
        DataLogManager.log(f"   Configuring drive motor: CAN ID: {config.drive_motor_id}")
        self.drive_motor = LazyCANSparkMax(config.drive_motor_id)
        self.drive_motor.setInverted(config.drive_motor_inverted)
        self.drive_pid_controller = self.drive_motor.getPIDcontroller()
        self.drive_pid_controller.setP(constants.DRIVE_KP)
        self.drive_pid_controller.setI(constants.DRIVE_KI)
        self.drive_pid_controller.setD(constants.DRIVE_KD)
        DataLogManager.log("   ...Done")
        DataLogManager.log(f"   Configuring steer motor: CAN ID: {config.steer_motor_id}")
        self.steer_motor = WPI_TalonSRX(config.steer_motor_id)
        self.steer_motor.setInverted(config.steer_motor_inverted)
        self.steer_motor.config_kP(constants.STEER_PID_IDX, constants.STEER_KP, 50)
        self.steer_motor.config_kI(constants.STEER_PID_IDX, constants.STEER_KI, 50)
        self.steer_motor.config_kD(constants.STEER_PID_IDX, constants.STEER_KD, 50)
        DataLogManager.log("   Configuring encoder")
        self.steer_motor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                                                      constants.STEER_PID_IDX,
                                                      50)
        self.steer_motor.setSensorPhase(config.sensor_phase_inverted)
        DataLogManager.log("   ...Done")
        DataLogManager.log("...Done")

    def get_swerve_angle(self) -> Rotation2d:
        steer_rotation = self.steer_motor.getSensorCollection().getPulseWidthPosition() / constants.STEERING_ENCODER_CPR
        swerve_angle = (steer_rotation / constants.STEER_GEARING_RATIO * constants.RADIANS_PER_REVOLUTION)
        return Rotation2d(swerve_angle)

    def set_swerve_angle(self, swerve_angle: Rotation2d) -> None:
        steer_encoder_pulses = swerve_angle.radians() / constants.RADIANS_PER_REVOLUTION * constants.STEER_GEARING_RATIO
        self.steer_motor.setSelectedSensorPosition(steer_encoder_pulses, constants.STEER_PID_IDX)

    def get_swerve_encoder_angle(self) -> Rotation2d:
        return Rotation2d(self.steer_motor.getSensorCollection().getPulseWidthPosition() / constants.STEERING_ENCODER_CPR * constants.RADIANS_PER_REVOLUTION)

    def set_swerve_angle_target(self, swerve_angle_target: Rotation2d) -> None:
        steer_encoder_pulses_target = (swerve_angle_target.radians() / constants.RADIANS_PER_REVOLUTION * constants.STEERING_ENCODER_CPR * constants.STEER_GEARING_RATIO)
        self.steer_motor.set(phoenix5.ControlMode.Position, steer_encoder_pulses_target)

    def get_wheel_linear_velocity(self) -> float:
        drive_encoder_rpm = self.drive_motor.getEncoder().getVelocity()
        wheel_linear_velocity = (drive_encoder_rpm * 60.0 * constants.WHEEL_RADIUS * constants.RADIANS_PER_REVOLUTION * constants.DRIVE_GEARING_RATIO)
        return wheel_linear_velocity

    def get_wheel_total_position(self) -> float:
        drive_encoder_revolutions = self.drive_motor.getEncoder().getPosition()
        drive_distance = drive_encoder_revolutions * constants.WHEEL_RADIUS * constants.RADIANS_PER_REVOLUTION * constants.DRIVE_GEARING_RATIO
        return drive_distance

    def set_wheel_linear_velocity_target(self, wheel_linear_velocity_target: float) -> None:
        drive_encoder_rpm = wheel_linear_velocity_target / constants.WHEEL_RADIUS / constants.RADIANS_PER_REVOLUTION * constants.DRIVE_GEARING_RATIO / 60.0
        self.drive_pid_controller.setReference(drive_encoder_rpm, LazyCANSparkMax.ControlType.kVelocity)

    def reset(self):
        self.set_swerve_angle(self.steer_motor.getSensorCollection().getPulseWidthPosition() / constants.STEERING_ENCODER_CPR)

class DriveSubsystem(Subsystem):
    class CoordinateMode(Enum):
        RobotRelative = auto()
        FieldRelative = auto()
        TargetRelative = auto()

    def __init__(self) -> None:
        Subsystem.__init__(self)
        self.setName(__class__.__name__)

        self.rotation_offset = 0
        self.front_left_module = WCPSwerveModule("Front Left Swerve Module",
                                                 SwerveModuleConfigParams(
                                                     constants.FRONT_LEFT_DRIVE_MOTOR_ID,
                                                     constants.FRONT_LEFT_DRIVE_MOTOR_INVERTED,
                                                     constants.FRONT_LEFT_STEER_MOTOR_ID,
                                                     constants.FRONT_LEFT_STEER_MOTOR_INVERTED,
                                                     constants.FRONT_LEFT_SWERVE_ENCODER_OFFSET,
                                                     constants.FRONT_LEFT_SENSOR_PHASE_INVERTED
                                                 ))
        self.front_right_module = WCPSwerveModule("Front Right Swerve Module",
                                                 SwerveModuleConfigParams(
                                                     constants.FRONT_RIGHT_DRIVE_MOTOR_ID,
                                                     constants.FRONT_RIGHT_DRIVE_MOTOR_INVERTED,
                                                     constants.FRONT_RIGHT_STEER_MOTOR_ID,
                                                     constants.FRONT_RIGHT_STEER_MOTOR_INVERTED,
                                                     constants.FRONT_RIGHT_SWERVE_ENCODER_OFFSET,
                                                     constants.FRONT_RIGHT_SENSOR_PHASE_INVERTED
                                                 ))
        self.back_left_module = WCPSwerveModule("Back Left Swerve Module",
                                                 SwerveModuleConfigParams(
                                                     constants.BACK_LEFT_DRIVE_MOTOR_ID,
                                                     constants.BACK_LEFT_DRIVE_MOTOR_INVERTED,
                                                     constants.BACK_LEFT_STEER_MOTOR_ID,
                                                     constants.BACK_LEFT_STEER_MOTOR_INVERTED,
                                                     constants.BACK_LEFT_SWERVE_ENCODER_OFFSET,
                                                     constants.BACK_LEFT_SENSOR_PHASE_INVERTED
                                                 ))
        self.back_right_module = WCPSwerveModule("Back Right Swerve Module",
                                                 SwerveModuleConfigParams(
                                                     constants.BACK_RIGHT_DRIVE_MOTOR_ID,
                                                     constants.BACK_RIGHT_DRIVE_MOTOR_INVERTED,
                                                     constants.BACK_RIGHT_STEER_MOTOR_ID,
                                                     constants.BACK_RIGHT_STEER_MOTOR_INVERTED,
                                                     constants.BACK_RIGHT_SWERVE_ENCODER_OFFSET,
                                                     constants.BACK_RIGHT_SENSOR_PHASE_INVERTED
                                                 ))

        self.modules = (
            self.front_left_module,
            self.front_right_module,
            self.back_left_module,
            self.back_right_module,
        )

        self.kinematics = SwerveDrive4Kinematics(
            constants.FRONT_LEFT_WHEEL_POSITION,
            constants.FRONT_RIGHT_WHEEL_POSITION,
            constants.BACK_LEFT_WHEEL_POSITION,
            constants.BACK_RIGHT_WHEEL_POSITION,
        )

        self.gyro = AHRS(SPI.Port.kMXP)
        while not self.gyro.isConnected():
            time.sleep(0.020)
        while self.gyro.isCalibrating():
            time.sleep(0.020)

        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            self.get_rotation(),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position(),
            ),
            Pose2d(),
            (0.1, 0.1, 0.1),
            (0.2, 0.2, 0.2),
        )

        self.odometry = SwerveDrive4Odometry(
            self.kinematics,
            self.get_rotation(),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position(),
            ),
            Pose2d()
        )
        self.print_timer = Timer()
        self.vx_limiter = SlewRateLimiter(constants.DRIVE_ACCEL_LIMIT)
        self.vy_limiter = SlewRateLimiter(constants.DRIVE_ACCEL_LIMIT)

    def get_robot_relative_speeds(self):
        return self.kinematics.toChassisSpeeds(self.get_module_states())

    def get_module_states(self):
        return (
            self.front_left_module.get_state(),
            self.front_right_module.get_state(),
            self.back_left_module.get_state(),
            self.back_right_module.get_state(),
        )

    def reset_swerve_modules(self):
        for module in self.modules:
            module.reset()
        self.reset_gyro(Pose2d())

    def set_odometry_posiiton(self, pose: Pose2d):
        self.rotation_offset = pose.rotation().degrees()
        self.reset_odometry_at_position(pose)

    def reset_gyro(self, pose: Pose2d):
        self.gyro.zeroYaw()
        self.rotation_offset = pose.rotation().degrees()
        self.reset_odometry_at_position(pose)

    def get_pose(self) -> Pose2d:
        translation = self.estimator.getEstimatedPosition().translation()
        rotation = self.get_rotation()
        return Pose2d(translation, rotation)

    def apply_states(self, module_states: Tuple[SwerveModuleState]) -> None:
        (
            front_left_state,
            front_right_state,
            back_left_state,
            back_right_state
        ) = SwerveDrive4Kinematics.desaturateWheelSpeeds(
            module_states, constants.MAX_WHEEL_LINEAR_VELOCITY
        )

        SmartDashboard.putNumberArray(
            "Swerve Expected States",
            [
                front_left_state.angle.radians(),
                front_left_state.speed,
                front_right_state.angle.radians(),
                front_right_state.speed,
                back_left_state.angle.radians(),
                back_left_state.speed,
                back_right_state.angle.radians(),
                back_right_state.speed,
            ],
        )
        self.front_left_module.apply_state(front_left_state)
        self.front_right_module.apply_state(front_right_state)
        self.back_left_module.apply_state(back_left_state)
        self.back_right_module.apply_state(back_right_state)

    def get_rotation(self) -> Rotation2d:
        return Rotation2d.fromDegrees(self.gyro.getYaw() + self.rotation_offset)

    def get_angular_velocity(self) -> float:
        if RobotBase.isSimulation():
            return SmartDashboard.getNumberArray(
                "Robot Velocity Array", [0, 0, 0]
            )[2]
        return (
            degreesToRadians(self.gyro.getRate())
        )

    def get_pitch(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-self.gyro.getPitch() + 180)

    def reset_odometry_at_position(self, pose: Pose2d):
        self.odometry.resetPosition(
            self.get_rotation(),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position(),
            ),
            pose,
        )

    def periodic(self) -> None:
        self.odometry.update(
            self.get_rotation(),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position(),
            ),
        )
        robot_pose = self.get_pose()

        SmartDashboard.putNumberArray(
            "Swerve Actual States",
            [
                self.front_left_module.get_swerve_encoder_angle().radians(),
                self.front_left_module.get_wheel_linear_velocity(),
                self.front_right_module.get_swerve_encoder_angle().radians(),
                self.front_right_module.get_wheel_linear_velocity(),
                self.back_left_module.get_swerve_encoder_angle().radians(),
                self.back_left_module.get_wheel_linear_velocity(),
                self.back_right_module.get_swerve_encoder_angle().radians(),
                self.back_right_module.get_wheel_linear_velocity(),
            ],
        )

        robot_pose_array = [robot_pose.X(), robot_pose.Y(), robot_pose.rotation().radians()]
        SmartDashboard.putNumberArray("Robot Pose Array", robot_pose_array)

        if self.print_timer.hasElapsed(constants.PRINT_PERIOD):
            DataLogManager.log(
                # pylint:disable-next=consider-using-f-string
                "r: {:.1f}, {:.1f}, {:.0f}* fl: {:.0f}* {:.1f} fr: {:.0f}* {:.1f} bl: {:.0f}* {:.1f} br: {:.0f}* {:.1f}".format(
                    robot_pose.X(),
                    robot_pose.Y(),
                    robot_pose.rotation().degrees(),
                    self.front_left_module.get_swerve_angle().degrees(),
                    self.front_left_module.get_wheel_linear_velocity(),
                    self.front_right_module.get_swerve_angle().degrees(),
                    self.front_right_module.get_wheel_linear_velocity(),
                    self.back_left_module.get_swerve_angle().degrees(),
                    self.back_left_module.get_wheel_linear_velocity(),
                    self.back_right_module.get_swerve_angle().degrees(),
                    self.back_right_module.get_wheel_linear_velocity(),
                )
            )

        self.estimator.updateWithTime(
            self.print_timer.getFPGATimestamp(),
            self.odometry.getPose().rotation(),
            (
                self.front_left_module.get_position(),
                self.front_right_module.get_position(),
                self.back_left_module.get_position(),
                self.back_right_module.get_position(),
            ),
        )
