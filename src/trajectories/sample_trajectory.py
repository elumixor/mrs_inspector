import math

from .trajectory import Trajectory
from .smooth_path import smooth_path
from .interpolate_heading import interpolate_heading
from .resample import resample


def sample_trajectory(poses: "list[tuple[float, float, float, float | None]]",
                      dt=0.2,
                      stop_duration=1.0,
                      lookahead=0.3,
                      sampling_step=0.1,
                      max_velocity=(2.0, 2.0, 1.0, 0.5),  # (x, y, z, heading)
                      max_acceleration=(2.0, 2.0, 1.0, 1.0)):  # (x, y, z, heading)
    if len(poses) <= 1:
        return Trajectory(poses)

    # Except for the first and last poses, stop for stop_duration seconds in each one.
    # This essentially means adding the pose ceil(stop_duration / dt) times if its heading is not None
    new_poses = [poses[0]]
    for pose in poses[1:-1]:
        if pose[3] is None:
            new_poses.append(pose)
        else:
            new_poses += [pose] * math.ceil(stop_duration / dt)
    new_poses.append(poses[-1])

    trajectory = Trajectory(new_poses)

    # If path smoothing is required, smooth the path
    trajectory = smooth_path(trajectory, lookahead, sampling_step)

    # Interpolate heading between waypoints
    trajectory = interpolate_heading(trajectory)

    # Parametrize trajectory and resample by the dt
    trajectory = resample(trajectory, max_velocity, max_acceleration, dt)

    return trajectory
