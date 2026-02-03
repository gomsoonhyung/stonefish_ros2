#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class RobotTrailPublisher(Node):
    """
    Publishes the robot's trajectory as a Path message for visualization in RViz.

    Subscribes to odometry and accumulates poses to create a trail showing
    where the robot has been.
    """

    def __init__(self):
        super().__init__('robot_trail_publisher')

        # Parameters
        self.declare_parameter('max_trail_length', 1000)
        self.max_trail_length = self.get_parameter('max_trail_length').value

        # Path to store robot trail
        self.trail = Path()
        self.trail.header.frame_id = 'GIRONA500/world_ned'

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/GIRONA500/dynamics_fixed',
            self.odom_callback,
            10
        )

        # Publisher for robot trail
        self.trail_pub = self.create_publisher(
            Path,
            '/GIRONA500/robot_trail',
            10
        )

        # Publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_trail)

        self.get_logger().info('Robot trail publisher started')
        self.get_logger().info(f'Max trail length: {self.max_trail_length} poses')

    def odom_callback(self, msg):
        """Add current pose to trail."""
        # Only add if position changed significantly (to avoid clutter)
        if len(self.trail.poses) > 0:
            last_pose = self.trail.poses[-1].pose
            dx = msg.pose.pose.position.x - last_pose.position.x
            dy = msg.pose.pose.position.y - last_pose.position.y
            dz = msg.pose.pose.position.z - last_pose.position.z
            distance = (dx*dx + dy*dy + dz*dz) ** 0.5

            # Skip if moved less than 5cm
            if distance < 0.05:
                return

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'GIRONA500/world_ned'
        pose_stamped.pose = msg.pose.pose

        self.trail.poses.append(pose_stamped)

        # Limit trail length
        if len(self.trail.poses) > self.max_trail_length:
            self.trail.poses.pop(0)

    def publish_trail(self):
        """Publish the accumulated trail."""
        if len(self.trail.poses) > 0:
            self.trail.header.stamp = self.get_clock().now().to_msg()
            self.trail_pub.publish(self.trail)


def main(args=None):
    rclpy.init(args=args)
    node = RobotTrailPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()