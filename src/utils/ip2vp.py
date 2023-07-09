import numpy as np

from utils import wrap_angle

from mrs_inspector.msg import InspectionPoint
from geometry_msgs.msg import Point


def ip2vp(ip: InspectionPoint, inspection_distance: float):
    """Converts InspectionPoint that needs to be inspected to a viewpoint (position and orientation of the UAV)"""
    x = ip.position.x + inspection_distance * np.cos(ip.heading)
    y = ip.position.y + inspection_distance * np.sin(ip.heading)
    z = ip.position.z
    position = Point(x, y, z)

    heading = wrap_angle(np.pi + ip.heading)

    return InspectionPoint(ip.id, position, heading, ip.possible_uavs)
