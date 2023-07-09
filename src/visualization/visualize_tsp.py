from __future__ import annotations

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from .functions import draw_path


def visualize_tsp(points_by_uav: dict[int, list[tuple[float, float, float, float | None]]]):
    for uav_id, points in points_by_uav.items():
        draw_path(
            points=[
                Pose(
                    position=Point(p[0], p[1], p[2]),
                    orientation=Quaternion(*quaternion_from_euler(0, 0, p[3] or 0))
                )
                for p in points
            ],
            topic=f"paths/uav{uav_id}",
        )
