#!/usr/bin/env python3
"""
GIRONA500 Sensor Monitor
Displays sensor data from GPS, IMU, DVL, and Pressure sensors
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, FluidPressure, NavSatFix
from nav_msgs.msg import Odometry
import sys
import os

# Try to import stonefish_ros2 messages
try:
    from stonefish_ros2.msg import DVL
    HAS_DVL = True
except ImportError:
    HAS_DVL = False


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        # Sensor data storage
        self.imu_data = None
        self.gps_data = None
        self.pressure_data = None
        self.dvl_data = None
        self.odom_data = None

        # Subscribers
        self.imu_sub = self.create_subscription(
            Imu, '/GIRONA500/imu_filter', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/GIRONA500/gps', self.gps_callback, 10)
        self.pressure_sub = self.create_subscription(
            FluidPressure, '/GIRONA500/pressure', self.pressure_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/GIRONA500/dynamics', self.odom_callback, 10)

        if HAS_DVL:
            self.dvl_sub = self.create_subscription(
                DVL, '/GIRONA500/dvl', self.dvl_callback, 10)

        # Timer for display update
        self.timer = self.create_timer(0.5, self.display_data)

        self.get_logger().info('GIRONA500 Sensor Monitor Started')

    def imu_callback(self, msg):
        self.imu_data = msg

    def gps_callback(self, msg):
        self.gps_data = msg

    def pressure_callback(self, msg):
        self.pressure_data = msg

    def dvl_callback(self, msg):
        self.dvl_data = msg

    def odom_callback(self, msg):
        self.odom_data = msg

    def display_data(self):
        # Clear screen
        os.system('clear')

        print("=" * 80)
        print(" " * 25 + "GIRONA500 SENSOR MONITOR")
        print("=" * 80)
        print()

        # IMU Data
        print("┌─ IMU (Inertial Measurement Unit) ─────────────────────────────────────┐")
        if self.imu_data:
            # Convert quaternion to euler angles (simplified)
            q = self.imu_data.orientation
            # For display purposes, just show quaternion
            print(f"│  Orientation (quat): x={q.x:+.3f}, y={q.y:+.3f}, z={q.z:+.3f}, w={q.w:+.3f}  │")
            av = self.imu_data.angular_velocity
            print(f"│  Angular Velocity:   x={av.x:+.3f}, y={av.y:+.3f}, z={av.z:+.3f} rad/s    │")
            la = self.imu_data.linear_acceleration
            print(f"│  Linear Accel:       x={la.x:+.3f}, y={la.y:+.3f}, z={la.z:+.3f} m/s²     │")
        else:
            print("│  No data available                                                      │")
        print("└────────────────────────────────────────────────────────────────────────┘")
        print()

        # GPS Data
        print("┌─ GPS (Global Positioning System) ─────────────────────────────────────┐")
        if self.gps_data:
            if self.gps_data.status.status >= 0:
                print(f"│  Latitude:   {self.gps_data.latitude:+.6f}°                                     │")
                print(f"│  Longitude:  {self.gps_data.longitude:+.6f}°                                    │")
                print(f"│  Status:     {'FIX' if self.gps_data.status.status >= 0 else 'NO FIX'}                                                    │")
            else:
                print("│  Status:     UNDERWATER (No GPS signal)                                │")
        else:
            print("│  No data available                                                      │")
        print("└────────────────────────────────────────────────────────────────────────┘")
        print()

        # Pressure Sensor
        print("┌─ Pressure Sensor ──────────────────────────────────────────────────────┐")
        if self.pressure_data:
            pressure_pa = self.pressure_data.fluid_pressure
            # Convert to depth (approximate: depth = (pressure - 101325) / (1025 * 9.81))
            depth_m = (pressure_pa - 101325) / (1025 * 9.81) if pressure_pa > 101325 else 0.0
            print(f"│  Pressure:   {pressure_pa:.1f} Pa                                        │")
            print(f"│  Depth:      {depth_m:.2f} m (approximate)                                  │")
        else:
            print("│  No data available                                                      │")
        print("└────────────────────────────────────────────────────────────────────────┘")
        print()

        # DVL Data
        if HAS_DVL:
            print("┌─ DVL (Doppler Velocity Log) ───────────────────────────────────────────┐")
            if self.dvl_data:
                v = self.dvl_data.velocity
                print(f"│  Velocity:   x={v.x:+.3f}, y={v.y:+.3f}, z={v.z:+.3f} m/s               │")
                print(f"│  Altitude:   {self.dvl_data.altitude:.2f} m                                      │")
            else:
                print("│  No data available                                                      │")
            print("└────────────────────────────────────────────────────────────────────────┘")
            print()

        # Odometry Data
        print("┌─ Odometry (Position & Velocity) ───────────────────────────────────────┐")
        if self.odom_data:
            p = self.odom_data.pose.pose.position
            v = self.odom_data.twist.twist.linear
            print(f"│  Position:   x={p.x:+.2f}, y={p.y:+.2f}, z={p.z:+.2f} m                  │")
            print(f"│  Velocity:   x={v.x:+.2f}, y={v.y:+.2f}, z={v.z:+.2f} m/s               │")
        else:
            print("│  No data available                                                      │")
        print("└────────────────────────────────────────────────────────────────────────┘")
        print()

        print("Press Ctrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    monitor = SensorMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
