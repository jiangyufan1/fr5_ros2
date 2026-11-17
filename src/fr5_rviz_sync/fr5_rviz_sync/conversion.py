"""No ROS imports: verify conversion of actual controller degree feedback."""
import math

JOINT_NAMES = tuple(f'j{i}' for i in range(1, 7))


def degrees_to_joint_positions(values):
    values = tuple(float(value) for value in values)
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError('Expected six finite controller joint angles in degrees')
    # Preserve feedback exactly; do not clamp to URDF limits or wrap revolutions.
    return tuple(math.radians(value) for value in values)
