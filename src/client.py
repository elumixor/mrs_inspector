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
                heading=5,
                possible_uavs=[1]
            ),
            InspectionPoint(
                id=2,
                position=Point(1, 1, 0.5),
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

        goal = InspectGoal(
            points=points,
            uav_ids=[1, 2],
            dt=0.2,
            inspection_distance=2.0,
            safety_distance=1.0,
            stop_duration=1.0,
            return_to_start=True,
        )
        self.client.send_goal(goal)
        self.client.wait_for_result()
        result = self.client.get_result()

        rospy.logwarn(f"Result: {result}")


if __name__ == "__main__":
    rospy.init_node("client")
    rospy.logwarn("client starting")

    client = InspectorClientNode()
    client.send_request()
