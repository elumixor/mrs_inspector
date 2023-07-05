import numpy as np

from mrs_inspector.msg import InspectionPoint

from rrt import rrt
from utils import ActionError, FreeSpace

from .lkh import lkh


def solve_single_tsp(viewpoints: "list[InspectionPoint]", free_space: FreeSpace):
    n = len(viewpoints)
    distances = np.zeros((n, n))
    paths = {}

    if len(viewpoints) < 1:
        raise ActionError("No viewpoints provided")

    if len(viewpoints) == 1:
        if viewpoints[0] not in free_space:
            raise ActionError("Inspection point is in obstacle space")

        vp = viewpoints[0]
        return [(vp.position.x, vp.position.y, vp.position.z, vp.heading)]

    # Find path between each pair of goals (a, b)
    for i, start in enumerate(viewpoints):
        for j, end in enumerate(viewpoints):
            if i == j:
                continue

            if i > j:
                paths[(i, j)] = paths[(j, i)][::-1]
                distances[i][j] = distances[j][i]
                continue

            # Estimate distances between the viewpoints
            # - Maybe we can improve it to use already existing connections?
            # - We can definitely also share it among all UAVs?
            path, distance = rrt(start, end, free_space)

            # Store paths/distances in matrices
            paths[(i, j)] = path
            distances[i][j] = distance

    # Compute TSP tour given the distances
    sequence = lkh(distances) if n > 2 else list(range(n))

    # Reconstruct path from sequence
    path = []
    n = len(distances)
    for a in range(n):
        b = (a + 1) % n
        a_idx = sequence[a]
        b_idx = sequence[b]

        # Join paths
        section = paths[(a_idx, b_idx)][:-1]

        # Add heading to the first point, and make it None everywhere else
        heading = viewpoints[a_idx].heading
        section = [(*p, heading if i == 0 else None) for i, p in enumerate(section)]

        # Add section to path
        path += section

    # Force flight to end point
    vp = viewpoints[sequence[-1]]
    vp = (vp.position.x, vp.position.y, vp.position.z, vp.heading)
    if path[-1] != vp:
        path.append(vp)

    # Close the loop by adding the first point again
    vp = viewpoints[sequence[0]]

    return path
