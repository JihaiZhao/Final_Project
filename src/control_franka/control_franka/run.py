import rclpy
from rclpy.node import Node
from moveit_wrapper.moveitapi import MoveItApi
from std_msgs.msg import Empty
from rclpy.callback_groups import ReentrantCallbackGroup
from control_franka_interfaces.action import MoveCartesian
from rclpy.action import ActionClient
import numpy as np
from control_franka_interfaces.srv import DelayTime
import csv


class Run(Node):
    def __init__(self):
        """Initializes the node.
        """
        super().__init__("run")

        self.action_client_move_cartesian = ActionClient(
            self, MoveCartesian, "move_cartesian_path", callback_group=ReentrantCallbackGroup()
        )

        # move it api to home robot
        self.moveit_api = MoveItApi(
            self,
            "panda_link0",
            "panda_hand_tcp",
            "panda_manipulator",
            "joint_states",
            "panda",
        )

        self.start_subscriber = self.create_subscription(
            Empty,
            "project_start",
            self.start_callback,
            10,
            callback_group=ReentrantCallbackGroup(),
        )

        # Delay service client
        self.delay_client = self.create_client(
            DelayTime, "delay", callback_group=ReentrantCallbackGroup()
        )
        self.start = False
        
        # self.timer = self.create_timer(0.1, self.timer_callback)

        self.filename = '/home/jihai/Jihai/Final_Project/src/trajectory_data.csv'

        # Initialize an empty list to store the data
        self.data = []

        self.end_point = get_data(self.filename, self.data)
        
        self.get_logger().info(f"Shape: {len(self.end_point)} All Points: {self.end_point}")

    async def start_callback(self, msg):
        """Callback function for the coffee_start subscriber. Triggers the make_coffee routine.
        """
        if not self.start:
            self.start = True
            await self.pick_place()

    async def pick_place(self):
        """Calls the actions required to make a cup of coffee in order.
        """
        # await self.moveit_api.plan_joint_async(
        # ["panda_joint1", "panda_joint2", "panda_joint3",
        #     "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"],
        # [-9.0/180*np.pi, 17.0/180*np.pi, -20.0/180*np.pi, -126.0/180*np.pi, 9.0/180*np.pi, 142.0/180*np.pi, 11.0/180*np.pi],
        # execute=True
        # ) 
        # await self.delay_client.call_async(DelayTime.Request(time=10.0))

        # go to observe position
        goal1 = MoveCartesian.Goal(
            num_points  = 100,
            endpoint_x  = [row[1] for row in self.end_point],
            endpoint_y  = [row[0] for row in self.end_point],
            start_outside=True,
            moving_frame="panda_hand_tcp",            
        )
        result = await self.action_client_move_cartesian.send_goal_async(goal1)
        await result.get_result_async()
        
        self.start = False

def run_entry(args=None):
    rclpy.init(args=args)
    run = Run()
    rclpy.spin(run)
    rclpy.shutdown()

def get_data(filename, data):
    # Open the CSV file
    try:
        with open(filename, mode='r') as file:
            csv_reader = csv.reader(file)

            # Iterate over each row in the file
            for row in csv_reader:
                try:
                    # Convert each value to float and store it in a list
                    data.append([float(value) for value in row])
                except ValueError as e:
                    print(f"Invalid value encountered: {e}")
    except FileNotFoundError:
        print(f"Could not open the file {filename}")

    # Now 'data' contains your parsed CSV data
    return data
