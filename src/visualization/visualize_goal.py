from __future__ import annotations

from mrs_inspector.msg import InspectGoal

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point

from utils import ip2vp
from .functions import draw_shapes, draw_arrows


def visualize_goal(goal: InspectGoal):
    points = goal.points

    draw_shapes(
        points=[p.position for p in points],
        shape="sphere",
        topic="inspection_points",
        color=ColorRGBA(0.3, 0.8, 0.6, 1.0),
        scale=0.5,
    )

    viewpoints = [ip2vp(p, goal.inspection_distance) for p in points]

    draw_shapes(
        points=[p.position for p in viewpoints],
        shape="sphere",
        topic="viewpoints",
        color=ColorRGBA(0.9, 0.2, 0.6, 1.0),
        scale=0.5,
    )

    arrow_points = []
    for ip, vp in zip(points, viewpoints):
        arrow_points.append(vp.position)
        arrow_points.append(ip.position)

    draw_arrows(
        points=arrow_points,
        topic="inspection_direction",
        color=ColorRGBA(0.9, 0.2, 0.6, 1.0),
    )
