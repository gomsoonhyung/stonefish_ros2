#!/usr/bin/env python3
"""
Custom AUV Sensor Monitor Node
Displays real-time sensor data from the Custom AUV
- Dynamics (position, velocity, orientation)
- Pressure (depth)
- DVL (velocity, altitude)
- IMU (orientation, angular velocity)
- GPS (position)
"""

import rclpy
from rclpy.node import Node
from stonefish_ros2.msg import DVL, Int32Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix, FluidPressure
import math


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('custom_auv_sensor_monitor')

        # Sensor data storage
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.orientation_rpy = [0.0, 0.0, 0.0]
        self.depth = 0.0
        self.dvl_velocity = [0.0, 0.0, 0.0]
        self.dvl_altitude = 0.0
        self.imu_angular_vel = [0.0, 0.0, 0.0]
        self.gps_lat = 0.0
        self.gps_lon = 0.0

        # Subscribers
        self.dynamics_sub = self.create_subscription(
            Odometry, '/CustomAUV/dynamics', self.dynamics_callback, 10)
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/CustomAUV/pressure', self.pressure_callback, 10)
        self.dvl_sub = self.create_subscription(
            DVL, '/CustomAUV/dvl', self.dvl_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/CustomAUV/imu', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/CustomAUV/gps', self.gps_callback, 10)

        # Display timer
        self.display_timer = self.create_timer(0.5, self.display_sensors)

        self.get_logger().info('Custom AUV Sensor Monitor Started')

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to roll, pitch, yaw (in degrees)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]

    def dynamics_callback(self, msg):
        """Process odometry (position, velocity, orientation)"""
        self.position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        self.velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ]
        self.orientation_rpy = self.quaternion_to_euler(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

    def pressure_callback(self, msg):
        """Process pressure sensor (depth calculation)"""
        # Depth = (Pressure - Atmospheric) / (Water Density * Gravity)
        # Atmospheric ≈ 101325 Pa, Water Density = 1025 kg/m³, g = 9.81 m/s²
        atmospheric_pressure = 101325.0
        water_density = 1025.0
        gravity = 9.81
        self.depth = max(0.0, (msg.fluid_pressure - atmospheric_pressure) / (water_density * gravity))

    def dvl_callback(self, msg):
        """Process DVL (velocity and altitude)"""
        self.dvl_velocity = [
            msg.velocity.x,
            msg.velocity.y,
            msg.velocity.z
        ]
        self.dvl_altitude = msg.altitude

    def imu_callback(self, msg):
        """Process IMU (angular velocity)"""
        self.imu_angular_vel = [
            math.degrees(msg.angular_velocity.x),
            math.degrees(msg.angular_velocity.y),
            math.degrees(msg.angular_velocity.z)
        ]

    def gps_callback(self, msg):
        """Process GPS (latitude, longitude)"""
        self.gps_lat = msg.latitude
        self.gps_lon = msg.longitude

    def display_sensors(self):
        """Display all sensor data in a formatted table"""
        print("\033[2J\033[H")  # Clear screen and move cursor to top
        print("=" * 80)
        print("Custom AUV Sensor Monitor".center(80))
        print("=" * 80)
        print()

        # Position & Orientation
        print("─── DYNAMICS (Odometry) ───".ljust(80))
        print(f"  Position (NED)    : X={self.position[0]:+7.2f}m  Y={self.position[1]:+7.2f}m  Z={self.position[2]:+7.2f}m (Depth: {self.depth:.2f}m)")
        print(f"  Velocity (Body)   : X={self.velocity[0]:+6.2f}m/s Y={self.velocity[1]:+6.2f}m/s Z={self.velocity[2]:+6.2f}m/s")
        print(f"  Orientation (RPY) : R={self.orientation_rpy[0]:+7.2f}° P={self.orientation_rpy[1]:+7.2f}° Y={self.orientation_rpy[2]:+7.2f}°")
        print()

        # DVL
        print("─── DVL (Doppler Velocity Log) ───".ljust(80))
        print(f"  Velocity          : X={self.dvl_velocity[0]:+6.2f}m/s Y={self.dvl_velocity[1]:+6.2f}m/s Z={self.dvl_velocity[2]:+6.2f}m/s")
        print(f"  Altitude (Bottom) : {self.dvl_altitude:6.2f}m")
        print()

        # IMU
        print("─── IMU (Inertial Measurement Unit) ───".ljust(80))
        print(f"  Angular Velocity  : Roll={self.imu_angular_vel[0]:+7.2f}°/s Pitch={self.imu_angular_vel[1]:+7.2f}°/s Yaw={self.imu_angular_vel[2]:+7.2f}°/s")
        print()

        # Pressure
        print("─── PRESSURE SENSOR ───".ljust(80))
        print(f"  Depth             : {self.depth:6.2f}m")
        print()

        # GPS
        print("─── GPS (Global Positioning System) ───".ljust(80))
        print(f"  Latitude          : {self.gps_lat:12.8f}°")
        print(f"  Longitude         : {self.gps_lon:12.8f}°")
        print()

        print("=" * 80)
        print("Press Ctrl+C to exit".center(80))


def main(args=None):
    rclpy.init(args=args)
    monitor = SensorMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\nShutting down sensor monitor...")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()