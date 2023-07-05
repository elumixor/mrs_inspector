from .trajectory import Trajectory
from .smooth_path import smooth_path
from .interpolate_heading import interpolate_heading
from .resample import resample


def sample_trajectory(poses: "list[tuple[float, float, float, float | None]]",
                      dt=0.2,
                      with_stops=False,
                      lookahead=0.3,
                      sampling_step=0.1,
                      max_velocity=(2.0, 2.0, 1.0, 0.5),  # (x, y, z, heading)
                      max_acceleration=(2.0, 2.0, 1.0, 1.0)):  # (x, y, z, heading)
    trajectory = Trajectory(poses)

    # if with_stops:
    #     # Interpolate heading between waypoints
    #     trajectory.waypoints = interpolate_heading(trajectory.waypoints)

    #     # Iterate through sequential waypoint pairs
    #     for w_idx in range(1, len(trajectory.waypoints)):
    #         pose_from = trajectory.waypoints[w_idx - 1]
    #         pose_to = trajectory.waypoints[w_idx]

    #         # Sample waypoint-to-waypoint line segments with stops at each start and end
    #         poses, _ = self.sampleStraightSegmentWithStops(pose_from, pose_to)

    #         # Add starting pose
    #         if w_idx == 1:
    #             poses = [pose_from] + poses

    #         trajectory.setSegment(w_idx - 1, poses)
    # else:

    # If path smoothing is required, smooth the path
    trajectory = smooth_path(trajectory, lookahead, sampling_step)

    # Interpolate heading between waypoints
    trajectory = interpolate_heading(trajectory)

    # Parametrize trajectory and resample by the dt
    trajectory = resample(trajectory, max_velocity, max_acceleration, dt)

    return trajectory
