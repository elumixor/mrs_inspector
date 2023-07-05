from __future__ import annotations

import numpy as np
from numpy.linalg import norm


class Trajectory:
    def __init__(self, points: "list[tuple[float, float, float, float | None]] | list[Pose]"):
        self.poses = [Pose(*point) for point in points]  # type: ignore

    def __getitem__(self, index: int):
        return self.poses[index]

    def __len__(self):
        return len(self.poses)

    def __iter__(self):
        yield from self.poses

    def __str__(self):
        return "Trajectory(\n\t" + "\n\t".join(str(pose) for pose in self.poses) + "\n)"

    def __repr__(self):
        return str(self)

    @property
    def total_distance(self):
        return sum(self.poses[i - 1].distance(self.poses[i]) for i in range(1, len(self)))

    def numpy(self):
        return np.array([pose.numpy() for pose in self.poses])


class Pose:
    def __init__(self, x: float, y: float, z: float, heading: "float | None" = None):
        self.position = np.array([x, y, z])
        self.heading = heading

    def distance(self, other: "Pose"):
        return float(norm(self.position - other.position))

    def __iter__(self):
        yield from self.position
        yield self.heading

    def __str__(self):
        x, y, z = self.position
        heading = f"{self.heading:.3f}" if self.heading is not None else "None"
        return f"Pose(x={x:.3f}, y={y:.3f}, z={z:.3f}, heading={heading})"

    def __repr__(self):
        return str(self)

    def numpy(self):
        return np.array([*self.position, self.heading])
