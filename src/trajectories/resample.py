from __future__ import annotations

import numpy as np
import toppra
import toppra.constraint as constraint
import toppra.algorithm as algo

from .trajectory import Trajectory

toppra.setup_logging("INFO")


def resample(trajectory: Trajectory,
             max_velocity: tuple[float, float, float, float],
             max_acceleration: tuple[float, float, float, float],
             dt: float):
    """Finds time parametrization throughout the given waypoints"""
    # Unwrap heading in all poses
    for k in range(1, len(trajectory)):
        previous_heading = trajectory[-1].heading
        current_heading = trajectory[k].heading

        assert previous_heading is not None
        assert current_heading is not None

        if abs(previous_heading - current_heading) >= np.pi:
            difference = current_heading - previous_heading
            heading_diff = np.arctan2(np.sin(difference), np.cos(difference))
            trajectory[k].heading = previous_heading + heading_diff

    # Compute time parametrization
    path = toppra.SplineInterpolator(np.linspace(0, 1, len(trajectory)), trajectory.numpy())
    pc_vel = constraint.JointVelocityConstraint(max_velocity)
    pc_acc = constraint.JointAccelerationConstraint(max_acceleration)
    instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel", gridpt_max_err_threshold=1e-6)
    parametrized = instance.compute_trajectory()

    assert parametrized is not None

    # Sample the parametrized trajectory
    ts = np.arange(0, parametrized.duration, dt)  # type: ignore
    trajectory = Trajectory(parametrized.eval(ts))  # type: ignore

    return trajectory
