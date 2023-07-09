from __future__ import annotations

import rospy

from mrs_inspector.msg import InspectionPoint

from utils import ip2vp
from free_space import FreeSpace

from .assign_viewpoints import assign_viewpoints
from .solve_single_tsp import solve_single_tsp


def solve_tsp(points: list[InspectionPoint],
              starts: dict[int, tuple[float, float, float, float] | None],
              free_space: FreeSpace,
              viewpoint_distance: float,
              return_to_start: bool):
    # Convert IPs to get the UAV position and heading
    points = [ip2vp(point, viewpoint_distance) for point in points]

    # Assign VP to UAVs
    vps_by_uav = assign_viewpoints(points, starts)

    # If return_to_start is True, we should add the starting point to the end of the sequence
    if return_to_start:
        for uav_id, vps in vps_by_uav.items():
            if len(vps) > 1:
                vps.append(vps[0])

    # Solve TSP for each UAV separately
    sequences: dict[int, list[tuple[float, float, float, float | None]]] = {}
    for uav_id, vps in vps_by_uav.items():
        message = f"Solving TSP for UAV {uav_id}. Viewpoints:"
        for vp in vps:
            x, y, z, heading = vp.position.x, vp.position.y, vp.position.z, vp.heading
            message += f"\n\t[{x:.2f}, {y:.2f}, {z:.2f}], {heading:.2f}"
        rospy.loginfo(message)

        sequences[uav_id] = solve_single_tsp(vps, free_space)

    return sequences
