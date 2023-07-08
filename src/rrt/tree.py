import numpy as np
from typing import NamedTuple, Union

Node = NamedTuple("Node", [("point", np.ndarray), ("parent", Union[np.ndarray, None]), ("cost", float)])


class Tree:
    def __init__(self, root: np.ndarray):
        self.nodes = {tuple(root): Node(root, None, 0)}

    def __getitem__(self, point: np.ndarray):
        return self.nodes[tuple(point)]

    def __iter__(self):
        return iter(self.nodes.keys())

    def add_node(self, point: np.ndarray, parent: np.ndarray, cost: float):
        self.nodes[tuple(point)] = Node(point, parent, cost)

    def find_path(self, end: np.ndarray):
        current_parent = self[end]

        path = []
        while current_parent is not None:
            path.append(current_parent.point)
            current_parent = self[current_parent.parent] if current_parent.parent is not None else None

        # Return path from start point to end point.
        path.reverse()

        return path
