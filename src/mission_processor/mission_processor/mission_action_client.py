import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from mission_action_interfaces.action import Mission
import time
import requests

class MissionActionClient(Node):
    """
    This class is an action client for the mission action server.
    It sends a goal to the action server when a new mission is received.
    """
    def __init__(self):
        """
        Initialize the MissionActionClient.
        """
        super().__init__("mission_action_client_node")
        self.action_client = ActionClient(self, Mission, "get_mission")
        self.get_logger().info("Mission Action Client is up and running.")
        # timer callback
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Timer is up and running.")
        self.mission = None

    def timer_callback(self):
        """
        Timer callback function.
        """
        self.get_logger().info("Timer callback")
        # get json data from GET REST api and save in mission
        response = requests.get('http://localhost:5000/get_mission')
        if self.mission is None:
            self.mission = response.json()
            self.get_logger().info(f"New mission: {self.mission}")
            self.send_goal(self.mission['mission']['description'])
        else:
            if self.mission['mission']['id'] != response.json()['mission']['id']:
                self.mission = response.json()
                self.get_logger().info(f"New mission: {self.mission}")
                self.send_goal(self.mission['mission']['description'])
            else:
                self.get_logger().info(f"Skipping Mission as the same mission is sent already: {self.mission}")

    def send_goal(self, mission):
        """
        Send a goal to the action server.
        """
        goal_msg = Mission.Goal()
        goal_msg.description = mission
        self.action_client.wait_for_server()
        self.get_logger().info("Sending goal")
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info("Goal sent")
        self.get_logger().info("Waiting for result")

def main(args=None):
    rclpy.init(args=args)
    mission_action_client = MissionActionClient()
    rclpy.spin(mission_action_client)
    mission_action_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
