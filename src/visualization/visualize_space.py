from __future__ import annotations

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from free_space import FreeSpace

from .functions import draw_points, draw_lines, draw_shapes


def visualize_space(free_space: FreeSpace):
    draw_points(
        points=[
            Point(x, y, z)
            for x in [free_space.min_x, free_space.max_x]
            for y in [free_space.min_y, free_space.max_y]
            for z in [free_space.min_z, free_space.max_z]
        ],
        topic="world_bounds/points",
        color=ColorRGBA(0.0, 1.0, 1.0, 1.0),
        scale=0.2,
    )

    draw_lines(
        points=[
            Point(x, y, z)
            for x in [free_space.min_x, free_space.max_x]
            for y in [free_space.min_y, free_space.max_y]
            for z in [free_space.min_z, free_space.max_z]
        ] + [
            Point(x, y, z)
            for x in [free_space.min_x, free_space.max_x]
            for z in [free_space.min_z, free_space.max_z]
            for y in [free_space.min_y, free_space.max_y]
        ] + [
            Point(x, y, z)
            for y in [free_space.min_y, free_space.max_y]
            for z in [free_space.min_z, free_space.max_z]
            for x in [free_space.min_x, free_space.max_x]
        ],
        topic="world_bounds/lines",
        color=ColorRGBA(0.0, 1.0, 1.0, 1.0),
        scale=0.1,
    )

    draw_shapes(
        points=[
            Point(x, y, z)
            for x, y, z in free_space.obstacles
        ],
        shape="sphere",
        topic="obstacles",
        color=ColorRGBA(0.3, 0.1, 0.1, 1.0),
        scale=free_space.safety_distance,
    )
