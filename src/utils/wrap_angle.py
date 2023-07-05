import numpy as np


def wrap_angle(angle: float):
    """Angle in interval [-pi, pi) (rad)"""
    return (angle + np.pi) % (2 * np.pi) - np.pi


def unwrap_angle(angle: float):
    """Angle in interval [0, 2pi) (rad)"""
    return (angle + 2.0 * np.pi) % (2.0 * np.pi)
