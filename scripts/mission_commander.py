#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mvp_msgs.srv import ChangeState
import sys


class MissionCommander(Node):
    """
    Simple mission commander to change MVP Helm states using service calls.

    Usage:
        ros2 run stonefish_ros2 mission_commander.py <state>

    States:
        - survey: Start waypoint following
        - teleop: Manual control mode
        - kill: Stop all behaviors
    """

    def __init__(self, target_state):
        super().__init__('mission_commander')

        self.target_state = target_state

        # Service client for state changes
        self.state_client = self.create_client(
            ChangeState,
            '/GIRONA500/mvp_helm/change_state'
        )

        self.get_logger().info('Mission Commander initialized')
        self.get_logger().info(f'Target state: {target_state}')

        # Wait for service to be available
        while not self.state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for change_state service...')

        # Call service
        self.call_change_state()

    def call_change_state(self):
        """Call the change_state service."""
        request = ChangeState.Request()
        request.state = self.target_state
        request.caller = 'mission_commander'

        self.get_logger().info(f'Calling change_state service: {self.target_state}')

        future = self.state_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        """Handle service response."""
        try:
            response = future.result()
            if response.status:
                self.get_logger().info(f'✓ State changed successfully to: {response.state.name}')
                self.get_logger().info(f'  Mode: {response.state.mode}')
                self.get_logger().info(f'  Available transitions: {response.state.transitions}')
            else:
                self.get_logger().error(f'✗ Failed to change state to: {self.target_state}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

        rclpy.shutdown()


def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run stonefish_ros2 mission_commander.py <state>")
        print("States: survey, teleop, kill")
        sys.exit(1)

    target_state = sys.argv[1]

    valid_states = ['survey', 'teleop', 'kill', 'start']
    if target_state not in valid_states:
        print(f"Error: Invalid state '{target_state}'")
        print(f"Valid states: {', '.join(valid_states)}")
        sys.exit(1)

    rclpy.init(args=args)
    node = MissionCommander(target_state)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()