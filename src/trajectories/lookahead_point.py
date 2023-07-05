import numpy as np

from utils import ActionError

from .trajectory import Pose, Trajectory


def lookahead_point(pose: Pose, path: Trajectory, lookahead: float, path_index: int):
    """Finds a point on the path at certain distance."""
    goal_candidates: list[Pose] = []
    pose_point = pose.position
    pose_heading = pose.heading

    while True:
        if path_index > 0:
            v_begin = path[path_index - 1]
            v_end = path[path_index]
        else:
            v_begin = pose
            v_end = path[path_index]

        v_begin_point = v_begin.position
        v_end_point = v_end.position

        # go to next segment if dist traveled is lower than lookahead dist and end of segment is closer than la dist
        dist_to_end = pose.distance(v_end)

        if dist_to_end < lookahead:
            if path_index == len(path) - 1:  # end of the path too close
                goal_candidates.append(v_end)
                break

            path_index = min(path_index + 1, len(path) - 1)
        else:
            intersections = _line_sphere_intersections(pose_point, lookahead, v_begin_point, v_end_point)
            goal_candidates = []

            for i in range(len(intersections)):
                intersection = intersections[i]
                goal_candidates.append(Pose(*intersection, pose_heading))  # type: ignore

            break

    if not goal_candidates:
        raise ActionError("Could not find lookahead point")

    if len(goal_candidates) == 1 and goal_candidates[0].distance(v_end) > dist_to_end:
        goal_candidates[0] = v_end

    dists = [gc.distance(v_end) for gc in goal_candidates]

    return goal_candidates[np.argmin(dists)], path_index


def _line_sphere_intersections(center: np.ndarray, sphere_radius: float, point1: np.ndarray, point2: np.ndarray):
    """Finds intersections between a line and a sphere"""
    vec = point2 - point1

    cx, cy, cz = center
    px, py, pz = point1
    p2x, p2y, p2z = point2

    A = vec @ vec
    B = 2.0 * (point1 @ vec - center @ vec)  # type: ignore
    C = px**2 - 2 * px * cx + cx**2 + py**2 - 2 * py * cy + cy**2 + pz**2 - 2 * pz * cz + cz**2 - sphere_radius**2

    # discriminant
    D = B**2 - 4 * A * C

    if (D < 0):
        return []

    t1 = (-B - (D ** 0.5)) / (2.0 * A)
    t2 = (-B + (D ** 0.5)) / (2.0 * A)

    if ((t1 > 1 or t1 < 0) and (t2 > 1 or t2 < 0)):
        return []

    solution1 = np.array([px * (1 - t1) + t1 * p2x, py * (1 - t1) + t1 * p2y, pz * (1 - t1) + t1 * p2z])
    solution2 = np.array([px * (1 - t2) + t2 * p2x, py * (1 - t2) + t2 * p2y, pz * (1 - t2) + t2 * p2z])

    if (not (t1 > 1 or t1 < 0) and (t2 > 1 or t2 < 0)):
        return [solution1]

    if ((t1 > 1 or t1 < 0) and not (t2 > 1 or t2 < 0)):
        return [solution2]

    if (D == 0):
        return [solution1]

    return [solution1, solution2]
