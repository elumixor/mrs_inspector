from __future__ import annotations

import numpy as np

from .trajectory import Trajectory


def resolve_collisions(trajectories: dict[int, Trajectory], dt=0.2, safety_distance=2.0):
    """Post-processes given trajectories such that there are no collisions"""

    delay_robot, delay_t = None, 0.0

    times = {uav: len(trajectory) * dt for uav, trajectory in trajectories.items()}
    lengths = {uav: trajectory.total_distance for uav, trajectory in trajectories.items()}

    # Decide which UAV should be delayed
    # This only works for two UAVs... Can we extend?
    keys = list(times.keys())

    delay_robot, non_delay_robot = np.argmin(list(times.values())), np.argmax(list(lengths.values()))
    delay_robot, non_delay_robot = keys[delay_robot], keys[non_delay_robot]

    if delay_robot != non_delay_robot:
        # check if the robot trajectories collide
        collision_flag, _ = trajectories_collide(trajectories[delay_robot], trajectories[non_delay_robot], safety_distance)

        while collision_flag:
            # delay the shorter-trajectory UAV at the start point by sampling period
            delay_step = dt
            delay_t += delay_step

            # Delay at start
            n_added = int(delay_step / dt)
            start_pose = trajectories[delay_robot][0]
            trajectories[delay_robot].poses = [start_pose] * n_added + trajectories[delay_robot].poses

            # keep checking if the robot trajectories collide
            collision_flag, _ = trajectories_collide(trajectories[delay_robot], trajectories[non_delay_robot], safety_distance)

    return trajectories


def trajectories_collide(trajectory_a: Trajectory, trajectory_b: Trajectory, safety_distance: float):
    samples_a, samples_b = trajectory_a, trajectory_b
    min_len = min([len(samples_a), len(samples_b)])
    for i in range(min_len):
        if samples_a[i].distance(samples_b[i]) < safety_distance:
            return True, i

    return False, 0
