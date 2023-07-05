from utils import wrap_angle

from .trajectory import Trajectory


def interpolate_heading(trajectory: Trajectory):
    """Interpolates linearly the UAV heading between waypoints."""

    assert trajectory[0].heading is not None, "Heading of first waypoints sample is None. Can't interpolate."
    assert trajectory[-1].heading is not None, "Heading of last waypoints sample is None. Last sample should equal the start."

    defined_headings = [i for i, pose in enumerate(trajectory) if pose.heading is not None]

    for i in range(1, len(defined_headings)):
        i_start = defined_headings[i - 1]
        i_end = defined_headings[i]

        if i_end - i_start == 1:
            continue

        heading_from = trajectory[i_start].heading
        heading_to = trajectory[i_end].heading

        assert heading_from is not None
        assert heading_to is not None

        heading_difference = wrap_angle(heading_to - heading_from)
        heading_from = wrap_angle(heading_from)

        segment = trajectory.poses[i_start:i_end + 1]
        segment_length = Trajectory(segment).total_distance

        current_distance = 0.0
        for j in range(1, len(segment)):
            current_distance += segment[j - 1].distance(segment[j])
            ratio = current_distance / segment_length
            trajectory[i_start + j].heading = wrap_angle(heading_from + heading_difference * ratio)

    return trajectory
