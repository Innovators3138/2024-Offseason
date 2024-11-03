import math

import phoenix5
import wpilib
from wpimath.controller import PIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from rev import CANSparkMax
from phoenix5 import WPI_TalonSRX, TalonSRXFeedbackDevice

import constants

class SwerveModule:
    def __init__(self, driving_can_id: int, steering_can_id: int, steering_encoder_offset: float, driving_inverted=False,
                 turning_inverted=False, label='') -> None:
        self.label = label
        self.desired_state = SwerveModuleState(0.0, Rotation2d())  # Initialize Desired State
        self.turning_output = 0

        """
        Initialize Driving Motor
        """
        self.driving_motor = CANSparkMax(driving_can_id, CANSparkMax.MotorType.kBrushless)
        self.driving_motor.restoreFactoryDefaults()
        self.driving_motor.setIdleMode(CANSparkMax.IdleMode.kBrake)
        self.driving_motor.setSmartCurrentLimit(constants.DRIVING_MOTOR_CURRENT_LIMIT)
        self.driving_motor.setInverted(driving_inverted)
        self.driving_motor.enableVoltageCompensation(constants.VOLTAGE_COMPENSATION)

        self.driving_encoder = self.driving_motor.getEncoder()
        self.driving_encoder.setPositionConversionFactor(constants.DRIVING_ENCODER_POSITION_FACTOR)
        self.driving_encoder.setVelocityConversionFactor(constants.DRIVING_ENCODER_VELOCITY_FACTOR)

        self.driving_pid_controller = self.driving_motor.getPIDController()
        self.driving_pid_controller.setFeedbackDevice(self.driving_encoder)
        self.driving_encoder.setPosition(0)

        """
        Initialize Turning Motor
        """
        self.steering_motor = WPI_TalonSRX(steering_can_id)
        self.steering_motor.clearStickyFaults()
        self.steering_motor.configFactoryDefault()
        self.steering_motor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.steering_motor.configPeakCurrentLimit(constants.STEERING_MOTOR_CURRENT_LIMIT)
        self.steering_motor.enableCurrentLimit(True)
        self.steering_motor.setInverted(turning_inverted)
        self.steering_motor.configVoltageCompSaturation(constants.VOLTAGE_COMPENSATION)
        self.steering_motor.enableVoltageCompensation(True)
        self.steering_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 50)
        self.steering_motor.setSensorPhase(constants.STEER_ENCODERS_INVERTED)
        self.steering_motor.configNominalOutputForward(0, 30)
        self.steering_motor.configNominalOutputReverse(0, 30)
        self.steering_motor.configPeakOutputForward(constants.STEERING_MAX_OUTPUT, 30)
        self.steering_motor.configPeakOutputReverse(constants.STEERING_MIN_OUTPUT, 30)

        self.steering_motor.selectProfileSlot(0, 0)
        self.steering_motor.config_kP(0, constants.STEERING_P, 30)
        self.steering_motor.config_kI(0, constants.STEERING_I, 30)
        self.steering_motor.config_kD(0, constants.STEERING_D, 30)
        self.steering_motor.config_kF(0, constants.STEERING_FF, 30)
        self.steering_motor.configPeakOutputForward(constants.STEERING_MAX_OUTPUT)
        self.steering_motor.configPeakOutputReverse(constants.STEERING_MIN_OUTPUT)
        self.steering_motor.configSensorTerm()


        self.steering_motor_position = self.steering_motor.getSensorCollection().getPulseWidthPosition() / constants.ENCODER_COUNTS_PER_REV * math.tau - steering_encoder_offset

    def get_steer_encoder(self):
        reverse_multiplier = -1 if constants.STEER_ENCODERS_INVERTED else 1
        return reverse_multiplier * self.steering_motor_position

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module
        """
        return SwerveModuleState(self.driving_encoder.getVelocity(),
                                 Rotation2d(self.get_steer_encoder()))

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driving_encoder.getPosition(),
                                    Rotation2d(self.get_steer_encoder()))

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:

        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle

        optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(self.get_steer_encoder()))

        if math.fabs(desiredState.speed) < 0.002:
            optimizedDesiredState.speed = 0
            optimizedDesiredState.angle = self.getState().angle

        self.driving_pid_controller.setReference(optimizedDesiredState.speed, constants.DRIVE_CONTROLLER_TYPE.ControlType.kVelocity)

        self.steering_motor.set(optimizedDesiredState.angle.radians() / math.tau * constants.ENCODER_COUNTS_PER_REV)
        self.turning_output = 0 if math.fabs(self.turning_output) < 0.01 else self.turning_output
        self.steering_motor.set(self.turning_output)

    def resetEncoders(self) -> None:
        self.driving_encoder.setPosition(0)

    def stop(self):
        pass

