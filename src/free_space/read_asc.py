import numpy as np


def read_asc(path: str):
    with open(path, "r") as f:
        obstacles = np.array([
            float(n)
            for line in f
            for n in line.strip().split()
        ]).reshape(-1, 3)

    return obstacles
