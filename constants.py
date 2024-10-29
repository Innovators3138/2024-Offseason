import math

from wpimath import units
from wpimath.units import inchesToMeters, rotationsPerMinuteToRadiansPerSecond, meters_per_second
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from rev import CANSparkMax
from wpimath.system.plant import DCMotor


"""
CAN Mapping
"""
LEFT_FRONT_DRIVE_CAN_ID = 1
RIGHT_FRONT_DRIVE_CAN_ID = 2
LEFT_BACK_DRIVE_CAN_ID = 3
RIGHT_BACK_DRIVE_CAN_ID = 4
LEFT_FRONT_STEER_CAN_ID = 5
RIGHT_FRONT_STEER_CAN_ID = 6
LEFT_BACK_STEER_CAN_ID = 7
RIGHT_BACK_STEER_CAN_ID = 8

"""
SOME IMPORTANT THINGS
"""
VOLTAGE_COMPENSATION = 12.0  # Volts

"""
MOTOR CONSTANTS

- Add All DC Motors We could want to Use, data taken from motors.vex.com
nominalVoltage: Volts
stallTorque: Newton-Meters
stallCurrent: Amps
freeCurrent: Amps
freeSpeed: Rad/sec
numMotors: int
"""
falcon500_motor = DCMotor(12.0, 4.69, 257.0, 1.5, rotationsPerMinuteToRadiansPerSecond(6380.0), 1)
neo_motor = DCMotor(12.0, 3.36, 166.0, 1.3, rotationsPerMinuteToRadiansPerSecond(5880.0), 1)
neo550_motor = DCMotor(12.0, 1.08, 111.0, 1.1, rotationsPerMinuteToRadiansPerSecond(11710.0), 1)
neovortex_motor = DCMotor(12.0, 3.6, 211.0, 3.6, rotationsPerMinuteToRadiansPerSecond(6784), 1)
cim_motor = DCMotor(12.0, 2.41, 131.0, 2.7, rotationsPerMinuteToRadiansPerSecond(5330.0), 1)
minicim_motor = DCMotor(12.0, 1.41, 89.0, 3.0, rotationsPerMinuteToRadiansPerSecond(5840.0), 1)
bag_motor = DCMotor(12.0, 0.43, 53.0, 1.8, rotationsPerMinuteToRadiansPerSecond(13180.0), 1)
vex775pro_motor = DCMotor(12.0, 0.71, 134.0, 0.7, rotationsPerMinuteToRadiansPerSecond(18730.0), 1)
krakenx60_motor = DCMotor(12.0, 7.09, 366.0, 2.0, rotationsPerMinuteToRadiansPerSecond(6000), 1)

DRIVE_CONTROLLER_TYPE = CANSparkMax

"""
SWERVE CONSTANTS - OVERALL
"""
MAX_SPEED_METERS_PER_SECOND = 1.0  # Might need to modify
MAX_ANGULAR_SPEED = 0.25 * math.tau  # radians per second
MAX_TOTAL_SPEED = math.sqrt(2) * MAX_ANGULAR_SPEED
MAGNITUDE_SLEW_RATE = 5  # hundred percent per second (1 = 100%)
ROTATIONAL_SLEW_RATE = 5 # hundred percent per second (1 = 100%)
INNER_DEADBAND = 0.08  # use deadbands for joystick transformations and keepangle calculations
OUTER_DEADBAND = 0.95
MINIMUM_ROTATION = MAX_ANGULAR_SPEED * INNER_DEADBAND

"""
SWERVE CONSTANTS - CHASSIS CONFIGURATIONS
"""
TRACK_WIDTH = units.inchesToMeters(24.0)  # Distance between centers of right and left wheels on the robot
WHEEL_BASE = units.inchesToMeters(24.0)  # Distance between centers of the front and back wheels on the robot

"""
This is key !  Here is where you get left and right correct
It's the minus signs in the 2nd column that swap l/r  - but it can still mess up
kinematics gets passed [self.frontLeft, self.frontRight, self.rearLeft, self.rearRight]
Front left is X+Y+, Front right is + -, Rear left is - +, Rear right is - -
this should be left as the convention, so match the above.  Then take care of turning issues with the
INVERSION OF THE TURN OR DRIVE MOTORS
"""
ORIENT = [(1, 1), (1, -1), (-1, 1), (-1, -1)]  # TODO: MAKE SURE ANGLE ENCODERS ARE CCW +

MODULE_POSITIONS = [
    Translation2d(ORIENT[0][0] * WHEEL_BASE / 2, ORIENT[0][1] * TRACK_WIDTH / 2),
    Translation2d(ORIENT[1][0] * WHEEL_BASE / 2, ORIENT[1][1] * TRACK_WIDTH / 2),
    Translation2d(ORIENT[2][0] * WHEEL_BASE / 2, ORIENT[2][1] * TRACK_WIDTH / 2),
    Translation2d(ORIENT[3][0] * WHEEL_BASE / 2, ORIENT[3][1] * TRACK_WIDTH / 2),
]

DRIVE_KINEMATICS = SwerveDrive4Kinematics(*MODULE_POSITIONS)

GYRO_REVERSED = False # Used in the swerve modules themselves to reverse the direction of the analog encoder
REVERSE_ANALOG_ENCODERS = False
DRIVE_MOTORS_INVERTED = False
TURN_MOTORS_INVERTED = True
ANALOG_ENCODER_ABS_MAX = 0.989  # Determined by filtering and watching as it flips from 1 to 0
# we pass this next one to the analog potentiometer object t0 determine the full range
ANALOG_ENCODER_SCALE_FACTOR = 1 / ANALOG_ENCODER_ABS_MAX

# absolute encoder values when wheels facing forward  - 20230322 CJH
# NOW IN RADIANS to feed right to the AnalogPotentiometer on the module
LF_ZERO_OFFSET = ANALOG_ENCODER_SCALE_FACTOR * math.tau * (0.829)  # rad
RF_ZERO_OFFSET = ANALOG_ENCODER_SCALE_FACTOR * math.tau * (0.783)  # rad
LB_ZERO_OFFSET = ANALOG_ENCODER_SCALE_FACTOR * math.tau * (0.436)  # rad
RB_ZERO_OFFSET = ANALOG_ENCODER_SCALE_FACTOR * math.tau * (0.829)  # rad
ANALOG_ENCODER_OFFSETS = {'lf':0.829, 'rf':0.783, 'lb':0.304, 'rb':0.986}  # use in sim

"""
SWERVE MODULE CONSTANTS
"""
DRIVING_MOTOR_FREE_SPEED_RPS = neo_motor.freeSpeed / 60
WHEEL_DIAMETER_METERS = units.inchesToMeters(4.0)
WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * math.pi
DRIVING_MOTOR_REDUCTION = 34.0 / 11.0 * 20.0 / 26.0 * 45.0 / 15.0  # ~7.13:1
DRIVE_WHEEL_FREE_SPEED_RPS = DRIVING_MOTOR_FREE_SPEED_RPS / DRIVING_MOTOR_REDUCTION
DRIVE_WHEEL_FREE_SPEED_MPS = DRIVE_WHEEL_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS

DRIVING_ENCODER_POSITION_FACTOR = WHEEL_CIRCUMFERENCE_METERS / DRIVING_MOTOR_REDUCTION
DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR / 60.0

TURNING_MOTOR_GEAR_RATIO = 396.0 / 35.0  # 11.3142:1

DRIVING_P = 0
DRIVING_I = 0
DRIVING_D = 0
DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS
DRIVING_MIN_OUTPUT = -0.96
DRIVING_MAX_OUTPUT = 0.96
SMART_POSITION_MAX_VELOCITY = 3  # m/s
SMART_POSITION_MAX_ACCEL = 2  # m/s/s

TURNING_P = 0.25
TURNING_I = 0.0
TURNING_D = 0.0
TURNING_FF = 0.0
TURNING_MIN_OUTPUT = -1.0
TURNING_MAX_OUTPUT = 1.0

DRIVING_MOTOR_CURRENT_LIMIT = 60  # amps
TURNING_MOTOR_CURRENT_LIMIT = 40  # amps


