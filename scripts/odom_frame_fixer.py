#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomFrameFixer(Node):
    """
    Fixes frame IDs in odometry messages from Stonefish to match MVP Control expectations.

    Stonefish publishes odometry with:
      - frame_id: world_ned
      - child_frame_id: GIRONA500/dynamics

    MVP Control expects:
      - frame_id: GIRONA500/world_ned (from tf_prefix + world_link_initial)
      - child_frame_id: GIRONA500/base_link (from tf_prefix + child_link_initial)

    This node subscribes to the original odometry, fixes the frame IDs,
    and republishes with corrected frames.
    """

    def __init__(self):
        super().__init__('odom_frame_fixer')

        # Subscribe to original odometry from Stonefish
        self.odom_sub = self.create_subscription(
            Odometry,
            '/GIRONA500/dynamics',
            self.odom_callback,
            10
        )

        # Publish corrected odometry
        self.odom_pub = self.create_publisher(
            Odometry,
            '/GIRONA500/dynamics_fixed',
            10
        )

        self.get_logger().info('Odometry frame fixer started')
        self.get_logger().info('Subscribing to: /GIRONA500/dynamics')
        self.get_logger().info('Publishing to: /GIRONA500/dynamics_fixed')
        self.get_logger().info('Frame ID fix: world_ned -> GIRONA500/world_ned')
        self.get_logger().info('Child frame ID fix: GIRONA500/dynamics -> GIRONA500/base_link')

    def odom_callback(self, msg):
        """Fix frame IDs and republish odometry."""
        # Create a new message with corrected frame IDs
        fixed_msg = Odometry()
        fixed_msg.header = msg.header
        fixed_msg.header.frame_id = 'GIRONA500/world_ned'
        fixed_msg.child_frame_id = 'GIRONA500/base_link'
        fixed_msg.pose = msg.pose
        fixed_msg.twist = msg.twist

        # Publish corrected odometry
        self.odom_pub.publish(fixed_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomFrameFixer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()