#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


class WaypointVisualizer(Node):
    """
    Visualizes waypoints and planned path in RViz.

    Publishes:
    - Waypoint markers (spheres)
    - Path line connecting waypoints
    """

    def __init__(self):
        super().__init__('waypoint_visualizer')

        # Waypoints from config (matching girona500_helm.yaml)
        self.waypoints = [
            {'x': 0.0, 'y': 0.0, 'z': 3.0},
            {'x': 10.0, 'y': 0.0, 'z': 3.0},
            {'x': 10.0, 'y': 10.0, 'z': 3.0},
            {'x': 0.0, 'y': 10.0, 'z': 3.0},
            {'x': 0.0, 'y': 0.0, 'z': 3.0},
        ]

        # Publishers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )

        self.path_pub = self.create_publisher(
            Marker,
            '/waypoint_path',
            10
        )

        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info('Waypoint visualizer started')
        self.get_logger().info(f'Visualizing {len(self.waypoints)} waypoints')

    def publish_markers(self):
        """Publish waypoint markers and path line."""
        # Waypoint spheres
        marker_array = MarkerArray()

        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'GIRONA500/world_ned'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = wp['z']
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.8
            marker.scale.y = 0.8
            marker.scale.z = 0.8

            # Color: yellow
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 0  # Never expire

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

        # Path line
        path_marker = Marker()
        path_marker.header.frame_id = 'GIRONA500/world_ned'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'path'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD

        # Add all waypoints as line points
        for wp in self.waypoints:
            point = Point()
            point.x = wp['x']
            point.y = wp['y']
            point.z = wp['z']
            path_marker.points.append(point)

        path_marker.scale.x = 0.2  # Line width

        # Color: green
        path_marker.color.r = 0.0
        path_marker.color.g = 1.0
        path_marker.color.b = 0.0
        path_marker.color.a = 1.0

        path_marker.lifetime.sec = 0  # Never expire

        self.path_pub.publish(path_marker)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()