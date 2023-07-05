import numpy as np
from .trajectory import Pose
from numpy.linalg import norm

from .trajectory import Trajectory
from .lookahead_point import lookahead_point


def smooth_path(trajectory: Trajectory, lookahead: float, sampling_step: float):
    path = [trajectory[0]]
    current = trajectory[0]

    path_index = 0
    while True:
        previous_index = path_index
        goal, path_index = lookahead_point(current, trajectory, lookahead, path_index)

        if path_index == len(trajectory) - 1 and trajectory[-1].distance(current) < sampling_step:
            path[-1].heading = trajectory[path_index].heading  # preserve last heading
            break

        # preserve assigned required heading
        if not previous_index == path_index:
            path[-1].heading = trajectory[previous_index].heading

        current = _simulate_step(current, goal, 1.0, sampling_step)
        current.heading = None
        path.append(current)

    return Trajectory(path)


def _simulate_step(pose_from: Pose, pose_to: Pose, speed: float, dt: float):
    position_from = pose_from.position
    position_to = pose_to.position
    heading = pose_from.heading

    vec = position_to - position_from
    distance = norm(vec)
    step_length = min(distance, speed * dt)  # type: ignore

    result = position_from + step_length * vec / distance

    return Pose(*result, heading)  # type: ignore
