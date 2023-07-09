import rospy
import numpy as np
import time

from mrs_inspector.msg import InspectionPoint

from utils import ActionError
from free_space import FreeSpace

from .tree import Tree


def rrt(start: InspectionPoint,
        end: InspectionPoint,
        free_space: FreeSpace,
        inflation=0.2,
        neighborhood=1.0,
        branch_size=2.0,
        discretization_factor=0.1,
        rrt_star=False,
        straighten=True,
        timeout=10.0,
        max_iterations=10000):

    def line_valid(p_from: np.ndarray, p_to: np.ndarray):
        vec = p_to - p_from
        distance = np.linalg.norm(vec)
        vec = vec / distance

        return all(
            (p_from + vec * i) in free_space
            for i in np.arange(0, distance, discretization_factor)
        ) and (p_to in free_space)

    def close_points(point: np.ndarray):
        for p in tree:
            if np.linalg.norm(point - p) < neighborhood and line_valid(point, p):
                yield p

    def straighten_path(path):
        if len(path) <= 2:
            return path

        if line_valid(path[0], path[-1]):
            return [path[0], path[-1]]

        i = len(path) // 2
        first, second = path[:i+1], path[i:]
        return straighten_path(first)[:-1] + straighten_path(second)

    start = start.position  # type: ignore
    start: np.ndarray = np.array([start.x, start.y, start.z])  # type: ignore
    end = end.position  # type: ignore
    end: np.ndarray = np.array([end.x, end.y, end.z])  # type: ignore
    if np.allclose(start, end):
        return [start], 0.0

    # Return immediately if start and end are in the obstacle space
    if start not in free_space:
        raise ActionError("Start point is in obstacle space")
    if end not in free_space:
        raise ActionError("End point is in obstacle space")

    rospy.loginfo(
        ("RRT*" if rrt_star else "RRT") +
        f": Searching for path from " +
        f"{start} to " +
        f"{end}.")

    mean = np.mean([start, end], axis=0)
    sigma = np.std([start, end], axis=0)

    sigma[sigma < 1e-3] = 0.1  # Inflate zero stddev
    tree = Tree(start)

    # build tree
    start_time = time.time()
    current_inflation = 0.0
    for _ in range(max_iterations):  # some NASA recommendation btw
        # Check if timeout has been reached
        if time.time() - start_time > timeout:
            raise ActionError(f"RRT tree did not build in the given timeout ({timeout} s).")

        # Sample the point
        for _ in range(10):
            point = np.random.normal(mean, current_inflation + sigma)
            if point in free_space:
                break
        else:
            raise ActionError("Could not sample a point in the free space.")

        # Find the closest point in the tree
        closest_point = min(tree, key=lambda p: np.linalg.norm(point - p))

        # Make sure there is at most branch_size distance between the points.
        vec = point - closest_point
        d = np.linalg.norm(vec)
        if d > branch_size:
            point = closest_point + vec * (branch_size / d)

        # If the line contains obstacles, try again with higher inflation.
        if not line_valid(point, closest_point):
            print(f"line invalid: {point} -> {closest_point}")
            current_inflation += inflation
            continue

        # RRT*: Choose neighbor which will provide the best cost.
        # Otherwise take the current one and use euclidean distance as cost.
        if rrt_star:
            # parent, cost = best_neighbor(point, closest_point)
            parent = closest_point
            cost = tree[closest_point].cost + np.linalg.norm(closest_point - point)

            for neighbor in close_points(point):
                neighbor_cost = tree[neighbor].cost + np.linalg.norm(point - neighbor)
                if neighbor_cost < cost:
                    cost, parent = neighbor_cost, neighbor
        else:
            parent, cost = closest_point, np.linalg.norm(point - closest_point)

        # Add the current node to the tree
        tree.add_node(point, parent, cost)
        rospy.loginfo(f"Added point: {point}")

        # RRT*: Rewire all neighbors
        if rrt_star:
            # Rewiring - if cost through given point is lower than its own, rewire it to go through that point.
            point_cost = tree[point].cost
            for neighbor in close_points(point):
                rewired_cost = point_cost + np.linalg.norm(neighbor - point)
                if rewired_cost < tree.get_cost(neighbor):
                    tree.add_node(neighbor, point, rewired_cost)

        # Check, whether end is reachable. If yes, stop the tree generation.
        if (np.linalg.norm(point - end) < branch_size) and line_valid(point, end):
            tree.add_node(end, point, tree[point].cost + np.linalg.norm(end - point))
            break
    else:
        raise ActionError("RRT tree did not build in the given timeout.")

    # find path
    rospy.loginfo("Tree build successfully. Retrieving path...")
    path = tree.find_path(end)

    # smooth the path
    if straighten:
        rospy.loginfo("Straightening path...")
        path = straighten_path(path)

    distance_total = sum(np.linalg.norm(path[i - 1] - path[i]) for i in range(1, len(path)))
    rospy.loginfo(f"Path found with length {distance_total:.5f}m:\n{str(np.array(path))}")

    return path, distance_total
