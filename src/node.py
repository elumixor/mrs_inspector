#!/usr/bin/env python3
from __future__ import annotations

import rospy
from tf.transformations import euler_from_quaternion

from mrs_inspector.msg import InspectAction, InspectGoal, InspectResult
from mrs_msgs.msg import TrajectoryReference, Reference, UavState
from mrs_msgs.srv import TrajectoryReferenceSrv
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from tsp import solve_tsp
from trajectories import sample_trajectory, resolve_collisions
from utils import ActionServer, ActionError, ip2vp
from free_space import FreeSpace
from visualization import visualize_space, visualize_goal, visualize_tsp, visualize_trajectories


class InspectorNode(ActionServer):
    def __init__(self):
        super().__init__("mrs_inspector", InspectAction)

        rospy.loginfo("Initializing...")

        # Create the space free from collisions
        self.free_space = FreeSpace(obstacles_filepath=rospy.get_param("~obstacles_file"),  # type: ignore
                                    bounds_filepath=rospy.get_param("~world_bounds_file"))  # type: ignore

        # Create the publishers of trajectories. We'll change them on each new request
        self.publishers: dict[int, rospy.Publisher] = {}
        # Create the services to call the control manager
        self.services: dict[int, rospy.ServiceProxy] = {}

        # Publish the world bounds and obstacles to the RViz
        visualize_space(self.free_space)

        rospy.loginfo("Initialization done. Waiting for requests")

    def on_request(self, goal: InspectGoal):
        """Handler of the only request of this node"""

        # Update visualizations
        visualize_goal(goal)
        visualize_space(self.free_space)

        points, uav_ids, dt, safety_distance, inspection_distance, return_to_start = self._process_request(goal)

        num_points = len(points)
        num_uavs = len(uav_ids)
        # Subscribe to the odometry topics of each UAV to get their starting positions and headings
        starts = self._get_starts(uav_ids)

        # Remove the old publishers
        for publisher in self.publishers.values():
            publisher.unregister()

        # Remove the old services
        for service in self.services.values():
            service.close()

        # For each UAV, create a publisher for the trajectory
        self.publishers = {
            uav: rospy.Publisher(f"~uav{uav}/inspector/trajectory", TrajectoryReference, queue_size=1, latch=True)
            for uav in uav_ids
        }
        # For each UAV, create a service proxy to call the control manager
        self.services = {
            uav: rospy.ServiceProxy(f"uav{uav}/control_manager/trajectory_reference", TrajectoryReferenceSrv)
            for uav in uav_ids
        }

        rospy.loginfo(f"Request received ({num_points} inspection points, {num_uavs} uavs). Solving TSP...")
        points_by_uav = solve_tsp(points, starts, self.free_space, inspection_distance, return_to_start)

        # Visualize trajectories
        visualize_tsp(points_by_uav)

        rospy.loginfo("TSP solved. Solving trajectories...")
        trajectories_by_uav = {uav: sample_trajectory(points, dt=dt) for uav, points in points_by_uav.items()}

        # Visualize trajectories
        visualize_trajectories(trajectories_by_uav)

        rospy.loginfo("Trajectories solved. Resolving collisions...")
        trajectories_by_uav = resolve_collisions(trajectories_by_uav, dt=dt, safety_distance=safety_distance)

        # Visualize trajectories
        visualize_trajectories(trajectories_by_uav)

        rospy.loginfo("Collisions resolved. Publishing trajectories...")

        # Publish the trajectories
        trajectories = {}
        for uav, trajectory in trajectories_by_uav.items():
            header = Header()
            header.stamp = rospy.Time.now()
            points = [Reference(Point(*p.position), p.heading) for p in trajectory]
            trajectories[uav] = TrajectoryReference(header=header,
                                                    points=points,
                                                    dt=0.2,
                                                    fly_now=True,
                                                    use_heading=True)

        for uav, trajectory in trajectories.items():
            self.publishers[uav].publish(trajectory)

        rospy.loginfo("Trajectories published and latched.")

        # Perform calls to the uav{id}/control_manager/trajectory_reference
        for uav, trajectory in trajectories.items():
            try:
                self.services[uav](trajectory)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Could not call service uav{uav}/control_manager/trajectory_reference. Reason: {e}")

        rospy.loginfo("Trajectories sent to control manager.")

        # Return the result
        return InspectResult(success=True)

    def _process_request(self, goal):
        points = goal.points
        uav_ids = goal.uav_ids
        inspection_distance = goal.inspection_distance
        safety_distance = goal.safety_distance
        dt = goal.dt

        self.free_space.safety_distance = safety_distance

        error_messages = []

        if len(uav_ids) <= 0:
            error_messages.append("No UAVs given")

        if len(set(uav_ids)) != len(uav_ids):
            error_messages.append("Duplicate UAVs given")

        if len(set([point.id for point in points])) != len(points):
            error_messages.append("Duplicate inspection points given")

        for point in points:
            if len(set(point.possible_uavs)) != len(point.possible_uavs):
                error_messages.append(f"IP {point.id} has duplicate UAVs")

            for uav in point.possible_uavs:
                if uav not in uav_ids:
                    error_messages.append(f"UAV {uav} is in IP {point.id} but not in starts")

            vp = ip2vp(point, inspection_distance)
            if vp not in self.free_space:
                error_messages.append(f"Viewpoint for IP {point.id} is in obstacle space")

        if dt <= 0:
            error_messages.append(f"dt must be positive, got {dt}")

        if inspection_distance <= 0:
            error_messages.append(f"inspection_distance must be positive, got {inspection_distance}")

        if safety_distance <= 0:
            error_messages.append(f"safety_distance must be positive, got {safety_distance}")

        if len(error_messages) > 0:
            raise ActionError("Error(s) in request:\n - " + "\n - ".join(error_messages))

        return points, uav_ids, dt, safety_distance, inspection_distance, goal.return_to_start

    def _get_starts(self, uav_ids: list[int]):
        starts: dict[int, tuple[float, float, float, float] | None] = {}
        for uav in uav_ids:
            topic = f"uav{uav}/odometry/uav_state"
            rospy.loginfo(f"Subscribing to {topic}")

            try:
                odometry: UavState = rospy.wait_for_message(topic, UavState, timeout=1.0)  # type: ignore
                position = odometry.pose.position
                orientation = odometry.pose.orientation
                _, _, heading = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
                starts[uav] = position.x, position.y, position.z, heading
            except rospy.exceptions.ROSException as e:
                starts[uav] = None
                rospy.logwarn(f"Could not get odometry for UAV {uav}. Will use the first point as a starting pose.\n" +
                              f"Reason: {e}")

        return starts


if __name__ == "__main__":
    rospy.init_node("mrs_inspector")

    # Create the node, it should read the config, read the obstacles, and await request
    InspectorNode()

    rospy.spin()
