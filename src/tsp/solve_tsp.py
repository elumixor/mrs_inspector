import rospy

from utils import ip2vp

from .assign_viewpoints import assign_viewpoints
from .solve_single_tsp import solve_single_tsp


def solve_tsp(points, starts, free_space, viewpoint_distance):
    # Convert IPs to get the UAV position and heading
    points = [ip2vp(point, viewpoint_distance) for point in points]

    # Assign VP to UAVs
    vps_by_uav = assign_viewpoints(points, starts)

    # Solve TSP for each UAV separately
    sequences = {}
    for uav_id, vps in vps_by_uav.items():
        rospy.loginfo(f"Solving TSP for UAV {uav_id}")
        sequences[uav_id] = solve_single_tsp(vps, free_space)

    return sequences
