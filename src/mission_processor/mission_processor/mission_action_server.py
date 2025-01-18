"""
Mission Action Server

This node is an action server for the mission action interface.
It receives a mission description as a goal and executes the mission.
The mission is simulated by a countdown of 10 seconds, and the
feedback is published every second.

"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from mission_action_interfaces.action import Mission
import time


class MissionActionServer(Node):
    """
    Mission Action Server

    This class is an action server for the mission action interface.
    It receives a mission description as a goal and executes the mission.
    The mission is simulated by a countdown of 10 seconds, and the
    feedback is published every second.
    """

    def __init__(self):
        """
        Initialize the Mission Action Server
        """
        super().__init__("mission_action_server_node")
        self.action_server = ActionServer(
            self,
            Mission,
            "get_mission",
            self.execute_callback
        )
        self.get_logger().info("Mission Action Server is up and running.")


    def execute_callback(self, goal_handle):
        """
        This function is called when a goal is received by the action server.
        """
        mission = goal_handle.request.description
        self.get_logger().info(f"Received mission: {mission}")
        
        feedback_msg = Mission.Feedback()
        # Update the feedback and publish for every countdown
        running_time = 10
        for i in range(running_time):
            feedback_msg.countdown = running_time- i
            self.get_logger().info(f"Countdown: {feedback_msg.countdown}")
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()
        result = Mission.Result()
        result.result = "Mission completed"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MissionActionServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
