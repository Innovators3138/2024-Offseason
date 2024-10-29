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
    def __init__(self, driving_can_id: int, turning_can_id: int, turning_encoder_offset: float, driving_inverted=False,
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
        self.turning_motor = WPI_TalonSRX(turning_can_id)
        self.turning_motor.clearStickyFaults()
        self.turning_motor.configFactoryDefault()
        self.turning_motor.setNeutralMode(phoenix5.NeutralMode.Coast)
        self.turning_motor.configPeakCurrentLimit(constants.TURNING_MOTOR_CURRENT_LIMIT)
        self.turning_motor.setInverted(turning_inverted)
        self.turning_motor.configVoltageCompSaturation(constants.VOLTAGE_COMPENSATION)
        self.turning_motor.enableVoltageCompensation(True)
        self.turning_motor.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 50)
        self.turning_motor.setSensorPhase(True)
        self.turning_motor.configNominalOutputForward(0, 30)
        self.turning_motor.configNominalOutputReverse(0, 30)
        self.turning_motor.configPeakOutputForward(constants.TURNING_MAX_OUTPUT, 30)
        self.turning_motor.configPeakOutputReverse(constants.TURNING_MIN_OUTPUT, 30)

        self.turning_motor.selectProfileSlot(0, 0)
        self.turning_motor.config_kP(0, constants.TURNING_P, 30)
        self.turning_motor.config_kI(0, constants.TURNING_I, 30)
        self.turning_motor.config_kD(0, constants.TURNING_D, 30)
        self.turning_motor.config_kF(0, constants.TURNING_FF, 30)

        self.turning_motor_position = self.turning_motor.getSensorCollection().getPulseWidthPosition()
        self.turning_PID_controller = PIDController(Kp=constants.TURNING_P, Ki=constants.TURNING_I, Kd=constants.TURNING_D)
        self.turning_PID_controller.enableContinuousInput(minimumInput=-math.pi, maximumInput=math.pi)

    def get_turn_encoder(self):
        analog_reverse_multiplier = -1 if constants.REVERSE_ANALOG_ENCODERS else 1
        return analog_reverse_multiplier * self.turning_motor_position

    def getState(self) -> SwerveModuleState:
        """Returns the current state of the module
        """
        return SwerveModuleState(self.driving_encoder.getVelocity(),
                                 Rotation2d(self.get_turn_encoder()))

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.driving_encoder.getPosition(),
                                    Rotation2d(self.get_turn_encoder()))

    def setDesiredState(self, desiredState: SwerveModuleState) -> None:

        correctedDesiredState = SwerveModuleState()
        correctedDesiredState.speed = desiredState.speed
        correctedDesiredState.angle = desiredState.angle

        optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState, Rotation2d(self.get_turn_encoder()))

        if math.fabs(desiredState.speed) < 0.002:
            optimizedDesiredState.speed = 0
            optimizedDesiredState.angle = self.getState().angle

        self.driving_pid_controller.setReference(optimizedDesiredState.speed, constants.DRIVE_CONTROLLER_TYPE.ControlType.kVelocity)

        self.turning_output = self.turning_PID_controller.calculate(self.get_turn_encoder(), optimizedDesiredState.angle.radians())
        self.turning_output = 0 if math.fabs(self.turning_output) < 0.01 else self.turning_output
        self.turning_motor.set(self.turning_output)

    def resetEncoders(self) -> None:
        self.driving_encoder.setPosition(0)

    def stop(self):
        pass

