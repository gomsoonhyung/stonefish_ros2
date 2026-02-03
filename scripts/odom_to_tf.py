#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTF(Node):
    """
    Converts odometry messages from Stonefish to TF transforms.

    Stonefish publishes odometry at /GIRONA500/dynamics but does not
    publish TF. This node bridges the gap by republishing the odometry
    as a TF transform from GIRONA500/world_ned to GIRONA500/Vehicle.
    """

    def __init__(self):
        super().__init__('odom_to_tf')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/GIRONA500/dynamics',
            self.odom_callback,
            10
        )

        self.get_logger().info('Odometry to TF bridge started')
        self.get_logger().info('Listening to: /GIRONA500/dynamics')
        self.get_logger().info('Publishing TF: GIRONA500/world_ned -> GIRONA500/base_link')

    def odom_callback(self, msg):
        """Convert odometry message to TF transform."""
        t = TransformStamped()

        # Header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'GIRONA500/world_ned'
        t.child_frame_id = 'GIRONA500/base_link'

        # Translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()