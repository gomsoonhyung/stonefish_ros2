#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from ament_index_python.packages import get_package_share_directory

class RobotStatePublisher(Node):
    def __init__(self):
        super().__init__('girona500_robot_state_publisher')

        # Read URDF file
        urdf_file = os.path.join(
            get_package_share_directory('stonefish_ros2'),
            'urdf',
            'girona500.urdf'
        )

        with open(urdf_file, 'r') as f:
            robot_desc = f.read()

        # Publish robot_description
        self.publisher = self.create_publisher(
            String,
            '/GIRONA500/robot_description',
            10
        )

        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_description)
        self.robot_desc = robot_desc

        self.get_logger().info('Robot description publisher started')
        self.get_logger().info(f'URDF file: {urdf_file}')

    def publish_description(self):
        msg = String()
        msg.data = self.robot_desc
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()