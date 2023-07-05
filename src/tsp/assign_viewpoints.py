import rospy
import numpy as np
from sklearn.cluster import KMeans

from mrs_inspector.msg import InspectionPoint


def assign_viewpoints(points, starts):
    viewpoints_by_uav = {
        # Create a list with the starting point only for now
        # (id is not really needed, but just to keep the coherent structure)
        start.id: [InspectionPoint(-1, start.position, start.heading, [start.id])]
        for start in starts
    }
    # IPs that can be inspected by multiple UAVs
    not_clustered = []

    for point in points:
        possible_uavs = point.possible_uavs

        # Multiple UAVs can inspect
        if len(possible_uavs) > 1:
            not_clustered.append(point)
            continue

        # Single UAV can inspect - add to the corresponding list
        viewpoints_by_uav[possible_uavs[0]].append(point)

    # Apply K-means clustering to separate the IPs
    rospy.loginfo(f"{len(not_clustered)} viewpoints to be clustered")
    clusters = cluster_vps(not_clustered, k=len(starts), assigned=[len(points) for points in viewpoints_by_uav.values()])

    # Add the clusters to the corresponding UAVs
    for i, cluster in enumerate(clusters):
        viewpoints_by_uav[starts[i].id] += cluster

    # Log the assigned viewpoints' indices
    viewpoints_str = ""
    for uav_id, viewpoints in viewpoints_by_uav.items():
        viewpoints_str += f"\n\tUAV {uav_id}: {[vp.id for vp in viewpoints[1:]]}"
    rospy.loginfo(f"Assigned viewpoints: {viewpoints_str}")

    return viewpoints_by_uav


def cluster_vps(viewpoints, k, assigned):
    """Clusters the viewpoints by positions into k clusters using K-means algorithm"""
    clusters = [[] for _ in range(k)]

    if not viewpoints:
        return clusters

    # If more UAVs than viewpoints, assign the VP to the UAV with the least assigned viewpoints
    # It's probably not the best solution, but it's ok for now
    if len(viewpoints) < k:
        while len(viewpoints) > 0:
            # Find the UAV with the least assigned viewpoints
            i = np.argmin(assigned)
            # Add the viewpoint to the UAV
            assigned[i] += 1
            # Add the viewpoint to the cluster
            clusters[i].append(viewpoints.pop())

        return clusters

    positions = np.array([(vp.position.x, vp.position.y, vp.position.z) for vp in viewpoints])

    kmeans = KMeans(n_clusters=k)
    labels = kmeans.fit_predict(positions)

    return [[vp for i, vp in enumerate(viewpoints) if labels[i] == r] for r in range(k)]
