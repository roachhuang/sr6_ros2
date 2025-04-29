#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from smallrobot_alex.action import PlanMovement

class PlannerActionServer(Node):

    def __init__(self):
        super().__init__('planner_action_server')
        self._action_server = ActionServer(
            self,
            PlanMovement,
            'plan_movement',
            self.execute_callback
        )

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal request: circle with radius {goal_handle.request.radius}")

        feedback_msg = PlanMovement.Feedback()
        for i in range(1, 11):
            feedback_msg.progress = i * 10.0
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"Feedback: {feedback_msg.progress}%")
            await rclpy.task.Future()  # small async sleep

        goal_handle.succeed()

        result = PlanMovement.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    action_server = PlannerActionServer()
    rclpy.spin(action_server)

if __name__ == '__main__':
    main()
