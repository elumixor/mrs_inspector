#!/usr/bin/env python

import rospy
import actionlib

from mrs_inspector.msg import InspectAction, InspectGoal, InspectionPoint, Start
from geometry_msgs.msg import Point


class InspectorClientNode:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("mrs_inspector", InspectAction)
        self.client.wait_for_server()

    def send_request(self):
        points = [
            InspectionPoint(
                id=1,
                position=Point(0, 0, 0.5),
                heading=0,
                possible_uavs=[1]
            ),
            InspectionPoint(
                id=2,
                position=Point(1, 0, 0.5),
                heading=0,
                possible_uavs=[1, 2]
            ),
            InspectionPoint(
                id=3,
                position=Point(3, 0, 0.5),
                heading=0,
                possible_uavs=[1]
            ),
        ]
        starts = [
            Start(
                id=1,
                position=Point(0, 0, 0.5),
                heading=0
            ),
            Start(
                id=2,
                position=Point(3, 3, 0.5),
                heading=0
            ),
        ]

        goal = InspectGoal(points=points, starts=starts)
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        rospy.logwarn(f"Result: {result}")


if __name__ == "__main__":
    rospy.init_node("client")
    rospy.logwarn("client starting")

    client = InspectorClientNode()
    client.send_request()
