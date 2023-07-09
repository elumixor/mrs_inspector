from __future__ import annotations

import rospy
from typing import Literal

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Header
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from nav_msgs.msg import Path


def draw_points(points: list[Point], topic: str, color: ColorRGBA | None = None, scale: float = 0.1):
    if color is None:
        color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    publisher = rospy.Publisher(f"rviz/mrs_inspector/{topic}", Marker, queue_size=1, latch=True)

    marker = Marker(
        header=Header(
            frame_id="map",
            stamp=rospy.Time.now(),
        ),
        ns="points",
        action=Marker.ADD,
        pose=Pose(
            orientation=Quaternion(w=1),
        ),
        id=0,
        type=Marker.POINTS,
        scale=Vector3(x=scale, y=scale),
        color=color,
        points=points
    )

    publisher.publish(marker)


def draw_lines(points: list[Point], topic: str, color: ColorRGBA | None = None, scale: float = 0.1):
    if color is None:
        color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    publisher = rospy.Publisher(f"rviz/mrs_inspector/{topic}", Marker, queue_size=1, latch=True)

    marker = Marker(
        header=Header(
            frame_id="map",
            stamp=rospy.Time.now(),
        ),
        ns="points",
        action=Marker.ADD,
        pose=Pose(
            orientation=Quaternion(w=1),
        ),
        id=0,
        type=Marker.LINE_LIST,
        scale=Vector3(x=scale),
        color=color,
        points=points
    )

    publisher.publish(marker)


def draw_shapes(points: list[Point],
                topic: str,
                shape: Literal["cube", "sphere"] = "cube",
                color: ColorRGBA | None = None,
                scale: float = 0.1):
    if color is None:
        color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    publisher = rospy.Publisher(f"rviz/mrs_inspector/{topic}", Marker, queue_size=1, latch=True)

    marker = Marker(
        header=Header(
            frame_id="map",
            stamp=rospy.Time.now(),
        ),
        ns="points",
        action=Marker.ADD,
        pose=Pose(
            orientation=Quaternion(w=1),
        ),
        id=0,
        type=Marker.CUBE_LIST if shape == "cube" else Marker.SPHERE_LIST,
        scale=Vector3(x=scale, y=scale, z=scale),
        color=color,
        points=points
    )

    publisher.publish(marker)


def draw_arrows(points: list[Point],
                topic: str,
                color: ColorRGBA | None = None,
                scale: float = 0.1,
                scale_head=0.2,
                head_length=0.1):
    if color is None:
        color = ColorRGBA(1.0, 0.0, 0.0, 1.0)

    publisher = rospy.Publisher(f"rviz/mrs_inspector/{topic}", MarkerArray, queue_size=1, latch=True)

    markers = []
    for i in range(len(points) // 2):
        start = points[i * 2]
        end = points[i * 2 + 1]

        marker = Marker(
            header=Header(
                frame_id="map",
                stamp=rospy.Time.now(),
            ),
            ns="points",
            action=Marker.ADD,
            pose=Pose(
                orientation=Quaternion(w=1),
            ),
            id=i,
            type=Marker.ARROW,
            scale=Vector3(x=scale, y=scale_head, z=head_length),
            color=color,
            points=[start, end]
        )

        markers.append(marker)

    publisher.publish(MarkerArray(markers))


def draw_path(points: list[Pose], topic: str, text=True):
    publisher_path = rospy.Publisher(f"rviz/mrs_inspector/{topic}/path", Path, queue_size=1, latch=True)

    path = Path(
        header=Header(
            frame_id="map",
            stamp=rospy.Time.now(),
        ),
        poses=[
            PoseStamped(
                header=Header(
                    frame_id="map",
                    stamp=rospy.Time.now(),
                ),
                pose=point
            )
            for point in points
        ]
    )

    publisher_path.publish(path)

    if not text:
        return

    publisher_text = rospy.Publisher(f"rviz/mrs_inspector/{topic}/text", MarkerArray, queue_size=1, latch=True)
    markers = []
    for i, point in enumerate(points):
        markers.append(
            Marker(
                header=Header(
                    frame_id="map",
                    stamp=rospy.Time.now(),
                ),
                ns="points",
                action=Marker.ADD,
                pose=Pose(
                    # Display below the position
                    position=Point(point.position.x, point.position.y, point.position.z - 0.5),
                    orientation=Quaternion(w=1),
                ),
                id=i,
                type=Marker.TEXT_VIEW_FACING,
                scale=Vector3(x=0.2, y=0.2, z=0.2),
                color=ColorRGBA(a=1.0),
                text=str(i),
            )
        )

    publisher_text.publish(MarkerArray(markers))
