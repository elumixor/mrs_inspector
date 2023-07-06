#!/usr/bin/env python3
import rospy


from mrs_inspector.msg import InspectAction, InspectResult
from mrs_msgs.msg import TrajectoryReference, Reference
from mrs_msgs.srv import TrajectoryReferenceSrv
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from tsp import solve_tsp
from trajectories import sample_trajectory, resolve_collisions
from utils import ActionServer, ActionError, FreeSpace, ip2vp


class InspectorNode(ActionServer):
    def __init__(self):
        super().__init__("mrs_inspector", InspectAction)

        rospy.loginfo("Initializing...")

        # Read some parameters
        self.viewpoint_distance: float = rospy.get_param("~viewpoint_distance")  # type: ignore

        # Create the space free from collisions
        self.free_space = FreeSpace(obstacles_filepath=rospy.get_param("~obstacles_file"),  # type: ignore
                                    bounds_filepath=rospy.get_param("~world_bounds_file"))  # type: ignore

        # Create the publishers of trajectories. We'll change them on each new request
        self.publishers: dict[int, rospy.Publisher] = {}
        # Create the services to call the control manager
        self.services: dict[int, rospy.ServiceProxy] = {}

        rospy.loginfo("Initialization done. Waiting for requests")

    def on_request(self, goal):
        """Handler of the only request of this node"""
        points = goal.points
        num_points = len(points)

        starts = goal.starts
        uavs = [start.id for start in starts]

        # Get the number of uavs from the points' inspectability
        num_uavs = len(uavs)
        self._validate_request(points, starts)

        # Remove the old publishers
        for publisher in self.publishers.values():
            publisher.unregister()

        # Remove the old services
        for service in self.services.values():
            service.close()

        # For each UAV, create a publisher for the trajectory
        self.publishers = {
            uav: rospy.Publisher(f"~uav{uav}/inspector/trajectory", TrajectoryReference, queue_size=1, latch=True)
            for uav in uavs
        }
        # For each UAV, create a service proxy to call the control manager
        self.services = {
            uav: rospy.ServiceProxy(f"uav{uav}/control_manager/trajectory_reference", TrajectoryReferenceSrv)
            for uav in uavs
        }

        rospy.loginfo(f"Request received ({num_points} inspection points, {num_uavs} uavs). Solving TSP...")
        points_by_uav = solve_tsp(points, starts, self.free_space, self.viewpoint_distance)

        rospy.loginfo("TSP solved. Solving trajectories...")
        trajectories_by_uav = {uav: sample_trajectory(points) for uav, points in points_by_uav.items()}

        rospy.loginfo("Trajectories solved. Resolving collisions...")
        trajectories_by_uav = resolve_collisions(trajectories_by_uav)

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
            self.services[uav](trajectory)

        rospy.loginfo("Trajectories sent to control manager.")

        # Return the result
        return InspectResult(success=True)

    def _validate_request(self, points, starts):
        error_messages = []

        if len(starts) <= 0:
            error_messages.append("No UAVs given")

        start_ids = [start.id for start in starts]
        set_starts = set(start_ids)
        if len(set_starts) != len(start_ids):
            error_messages.append("Duplicate UAVs given")

        if len(set([point.id for point in points])) != len(points):
            error_messages.append("Duplicate inspection points given")

        for point in points:
            if len(set(point.possible_uavs)) != len(point.possible_uavs):
                error_messages.append(f"IP {point.id} has duplicate UAVs")

            for uav in point.possible_uavs:
                if uav not in start_ids:
                    error_messages.append(f"UAV {uav} is in IP {point.id} but not in starts")

            vp = ip2vp(point, self.viewpoint_distance)
            if vp not in self.free_space:
                error_messages.append(f"Viewpoint for IP {point.id} is in obstacle space")

        for start in starts:
            if start not in self.free_space:
                error_messages.append(f"Start for UAV {start.id} is in obstacle space")

        if len(error_messages) > 0:
            raise ActionError("Error(s) in request:\n - " + "\n - ".join(error_messages))


if __name__ == "__main__":
    rospy.init_node("mrs_inspector")

    # Create the node, it should read the config, read the obstacles, and await request
    InspectorNode()

    rospy.spin()
