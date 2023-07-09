from __future__ import annotations

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

from trajectories import Trajectory

from .functions import draw_path


def visualize_trajectories(points_by_uav: dict[int, Trajectory]):
    for uav_id, trajectory in points_by_uav.items():
        print(list(trajectory))
        draw_path(
            points=[
                Pose(
                    position=Point(*p.position),
                    orientation=Quaternion(*quaternion_from_euler(0, 0, p.heading))
                )
                for p in trajectory
            ],
            topic=f"trajectories/uav{uav_id}",
            text=False
        )
