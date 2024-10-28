"""
Performs the spiral pouring action.

Action Servers:
  + pour_action (botrista_interfaces/PourAction) - Action for pouring in a spiral motion

"""


import rclpy
from rclpy.node import Node
import math
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

# Object Importing
from moveit_wrapper.moveitapi import MoveItApi
from geometry_msgs.msg import Point, Quaternion, Pose
from rclpy.action import ActionServer
from std_msgs.msg import Empty
from control_franka_interfaces.action import MoveCartesian
import csv

class Move_cartesian(Node):
    """Perform the spiral pouring action."""

    def __init__(self):
        super().__init__(node_name="move_cartesian")
        self.path = None
        self.cb = ReentrantCallbackGroup()

        # Creating tf listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_parent_frame = "panda_link0"

        # Moveit Wrapper Object
        self.moveit = MoveItApi(self,
                                "panda_link0",
                                "panda_hand_tcp",
                                "panda_manipulator",
                                "joint_states",
                                "panda")

        # Creating action server
        self._action_server = ActionServer(self,
                                           MoveCartesian,
                                           'move_cartesian_path',
                                           self.move_cartesian_callback,
                                           callback_group=self.cb)
        self.project_publisher = self.create_publisher(
            Empty, "project_start", qos_profile=10
        )
        
        # Timer intializing
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.save_csv = True
        
    async def timer_callback(self):
        self.project_publisher.publish(Empty())
        
        
    async def move_cartesian_callback(self, goal_handle):
        """
        Perform the spiral pour action.

        Args:
            goal_handle (PourAction.Goal) -- goal of the pour action

        Returns:
            PourAction.Result -- Result of the action
        """
        result = MoveCartesian.Result()
        feedback = MoveCartesian.Feedback()
        req = goal_handle.request

        currPose = await self.moveit.get_end_effector_pose()
        startPoint = currPose.pose.position
        startOre = currPose.pose.orientation

        endPoints = []
        for i in range(len(req.endpoint_x)):
            endPoint = Point()
            endPoint.x = req.endpoint_x[i]
            endPoint.y = req.endpoint_y[i]
            endPoint.z = 0.2
            endPoints.append(endPoint)
        
        # self.get_logger().info(f"End Points: {len(endPoints)}")
        # self.get_logger().info(f"End Points: {endPoints[0].x}")
        
        # Calculating path
        feedback.stage = "Calculating path"
        goal_handle.publish_feedback(feedback)
        
        waypoints = get_waypoints(startPoint,
                                         startOre,
                                         endPoints,
                                         req.num_points,
                                         req.start_outside)
        
        self.get_logger().info(f"WAY Points: {waypoints}")
        
        if self.save_csv:
            save_waypoints_csv(waypoints)
            self.save_csv = False

        # Planning
        feedback.stage = "Planning path"
        goal_handle.publish_feedback(feedback)
        planned_traj = await self.moveit.create_cartesian_path(waypoints)

        # Executing path
        feedback.stage = "Executing path"
        goal_handle.publish_feedback(feedback)
        self.moveit.execute_trajectory(planned_traj.trajectory)

        goal_handle.succeed()
        result.status = True
        return result


def main(args=None):
    rclpy.init(args=args)
    node = Move_cartesian()
    rclpy.spin(node)
    rclpy.shutdown()


def get_waypoints(startPoint: Point,
                         startOre: Quaternion,
                         endPoints: list[Point],
                         numPoints: int,
                         flipStart: bool = False) -> (list[Pose], float):
    """
    Create a spiral path given parameters

    Args:
        startPoint (geometry_msgs/Point) -- Starting point
        startOre (geometry_msgs/Quaternion) -- Starting Orientation
        numPoints (int) -- number of points used to build the path
        maxRadius (float) -- distance from end of spiral to origin in cm
        loops (float) -- number of loops for the spiral to go through
        flipStart (bool) -- Start at the end of the spiral instead of the center (default: {False})

    Returns:
        A list of waypoints

    """
    poseList = []
    for i in range(len(endPoints)):
        count = 0
        dx = (endPoints[i].x - startPoint.x)/numPoints
        dy = (endPoints[i].y - startPoint.y)/numPoints
        x_n = startPoint.x
        y_n = startPoint.y

        # Create poses for each point along the spiral
        while count < numPoints:
            x_n += dx
            y_n += dy
            z_n = startPoint.z

            poseList.append(Pose(position=Point(x=x_n,
                                                y=y_n,
                                                z=z_n),
                                orientation=startOre))

            count += 1
        
        startPoint = endPoints[i]
        

        if flipStart:
            poseList.reverse

    return poseList

def save_waypoints_csv(waypoints: list[Pose]):
    # Function to save the extracted data to a CSV file
    with open("franka_trajectory.csv", mode='w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # Write the O_T_EE[12] and O_T_EE[13] values
        for entry in waypoints:
            writer.writerow([entry.position.x, entry.position.y])