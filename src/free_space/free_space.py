import yaml
import numpy as np
from sklearn.neighbors import KDTree

from mrs_inspector.msg import InspectionPoint, Start
from geometry_msgs.msg import Point

from .read_dae import read_dae
from .read_asc import read_asc


class FreeSpace:
    def __init__(self, obstacles_filepath: str, bounds_filepath: str, safety_distance=2.0):
        with open(bounds_filepath, "r") as f:
            bounds = yaml.safe_load(f)

        self.min_x, self.max_x = bounds["x"]
        self.min_y, self.max_y = bounds["y"]
        self.min_z, self.max_z = bounds["z"]

        if obstacles_filepath.endswith(".dae"):
            self.obstacles = read_dae(obstacles_filepath)
        elif obstacles_filepath.endswith(".asc"):
            self.obstacles = read_asc(obstacles_filepath)
        else:
            raise RuntimeError(f"Unknown file type: {obstacles_filepath}")

        # setup KD tree for collision queries
        self.kd_tree = KDTree(self.obstacles)
        self.safety_distance = safety_distance

    def __contains__(self, point):
        if isinstance(point, InspectionPoint) or isinstance(point, Start):
            point = point.position
            point = np.array([point.x, point.y, point.z])
        elif isinstance(point, Point):
            point = np.array([point.x, point.y, point.z])
        elif isinstance(point, tuple) and len(point) == 3:
            point = np.array(point)
        elif isinstance(point, list) and len(point) == 3:
            point = np.array(point)

        if isinstance(point, np.ndarray) and point.shape == (3,):
            x, y, z = point

            if self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y and self.min_z <= z <= self.max_z:
                return self.kd_tree.query(point.reshape(1, 3), k=1)[0] > self.safety_distance

            return False
        else:
            return False
