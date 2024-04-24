from math import tau, floor
from wpimath.geometry import Rotation2d


def optimize_angle(current_angle: Rotation2d, target_angle: Rotation2d) -> Rotation2d:
    current_angle = current_angle.radians()

    closest_full_rotation = (
            floor(abs(current_angle / tau)) * (-1 if current_angle < 0 else 1) * tau
    )

    current_optimal_angle = target_angle.radians() + closest_full_rotation - current_angle

    potential_new_angles = [
        current_optimal_angle,
        current_optimal_angle - tau,
        current_optimal_angle + tau,
    ]  # closest other options

    delta_angle = tau  # max possible error, a full rotation!
    for potential_angle in potential_new_angles:
        if abs(delta_angle) > abs(potential_angle):
            delta_angle = potential_angle

    return Rotation2d(delta_angle + current_angle)