import functools
import math
import operator
import typing
from wpimath.geometry import Pose3d, Rotation2d, Rotation3d, Translation2d, Pose2d

number = typing.Union[float, int]


def clamp(inputValue: float, minimum: float, maximum: float) -> float:
    return max(min(inputValue, maximum), minimum)


def normalize_rotation(input_rotation: Rotation2d) -> Rotation2d:
    """
    Normalize the given rotation to the range [-pi, pi)
    """
    input_angle = input_rotation.radians()
    return Rotation2d(
        input_angle - 2 * math.pi * math.floor((input_angle + math.pi) / (2 * math.pi))
    )


def translation_from_distance_and_rotation(
    distance: float, rotation: Rotation2d
) -> Translation2d:
    return Translation2d(distance * rotation.cos(), distance * rotation.sin())


def rotation_from_translation(translation: Translation2d) -> Rotation2d:
    return Rotation2d(math.atan2(translation.Y(), translation.X()))


def rotate_around_point(
    pose: Pose2d, position: Translation2d, rotation: Rotation2d
) -> Pose2d:
    delta_translation = pose.translation() - position
    new_rotation = rotation + pose.rotation()

    rotated_translation = translation_from_distance_and_rotation(
        delta_translation.distance(Translation2d()),
        rotation_from_translation(delta_translation) + rotation,
    )

    return Pose2d(rotated_translation + position, new_rotation)


def map_range(
    value: number,
    input_min: number,
    input_max: number,
    output_min: number,
    output_max: number,
):
    return (value - input_min) * (output_max - output_min) / (
            input_max - input_min
    ) + output_min


def average(averageable_list: typing.List[typing.Any], initial: typing.Any):
    length = len(averageable_list)
    return functools.reduce(operator.add, averageable_list, initial) / length


def add_pose2d(a: Pose2d, b: Pose2d):
    return Pose2d(
        a.X() + b.X(),
        a.Y() + b.Y(),
        Rotation2d(a.rotation().radians() + b.rotation().radians()),
    )


def pose3d_from2d(pose: Pose2d) -> Pose3d:
    return Pose3d(pose.X(), pose.Y(), 0, Rotation3d(0, 0, pose.rotation().radians()))


def translation_dot_product(a: Translation2d, b: Translation2d) -> float:
    return a.x * b.x + a.y * b.y


def point_in_rectangle(
    rect: typing.Tuple[Translation2d, Translation2d, Translation2d, Translation2d],
    point: Translation2d,
) -> bool:
    a, b, c, _ = rect
    ab = a - b
    am = a - point
    bc = b - c
    bm = b - point
    dot = translation_dot_product

    dot_abam = dot(ab, am)
    dot_abab = dot(ab, ab)
    dot_bcbm = dot(bc, bm)
    dot_bcbc = dot(bc, bc)
    return 0 <= dot_abam <= dot_abab and 0 <= dot_bcbm <= dot_bcbc


def point_in_circle(p1: Translation2d, c: Translation2d, r: float) -> bool:
    return (p1 - c).norm() <= r