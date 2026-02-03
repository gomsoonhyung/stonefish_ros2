#!/usr/bin/env python3
"""
GIRONA500 AUV Keyboard Teleoperation Node
Controls the GIRONA500 AUV using keyboard inputs (WASD, Q/Z for vertical)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select


class Girona500Teleop(Node):
    def __init__(self):
        super().__init__('girona500_teleop')

        # Publisher for each thruster
        self.thruster_surge_port_pub = self.create_publisher(
            Float64, '/GIRONA500/ThrusterSurgePort/setpoint', 10)
        self.thruster_surge_starboard_pub = self.create_publisher(
            Float64, '/GIRONA500/ThrusterSurgeStarboard/setpoint', 10)
        self.thruster_heave_bow_pub = self.create_publisher(
            Float64, '/GIRONA500/ThrusterHeaveBow/setpoint', 10)
        self.thruster_heave_stern_pub = self.create_publisher(
            Float64, '/GIRONA500/ThrusterHeaveStern/setpoint', 10)
        self.thruster_sway_pub = self.create_publisher(
            Float64, '/GIRONA500/ThrusterSway/setpoint', 10)

        # Thruster control parameters
        self.max_thrust = 1.0  # Normalized setpoint range: -1.0 to 1.0
        self.thrust_increment = 0.1

        # Current thrust values
        self.surge_thrust = 0.0
        self.sway_thrust = 0.0
        self.heave_thrust = 0.0
        self.yaw_thrust = 0.0  # For rotation (left/right turn)

        self.get_logger().info('GIRONA500 Keyboard Teleoperation Node Started')
        self.print_instructions()

        # Timer for publishing thruster commands
        self.timer = self.create_timer(0.1, self.publish_thrusters)

    def print_instructions(self):
        msg = """
        ============================================
        GIRONA500 AUV Keyboard Teleoperation
        ============================================
        Controls:
          W/S : Forward / Backward (Surge)
          A/D : Left / Right (Sway)
          E/R : Turn Right / Turn Left (Yaw)
          Q/Z : Up / Down (Heave)
          X   : Stop all thrusters
          ESC : Exit

        Current Thrust Values will be displayed below
        ============================================
        """
        print(msg)

    def get_key(self):
        """Get keyboard input in non-blocking mode"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard input and update thrust values"""
        if key == 'w' or key == 'W':
            self.surge_thrust = min(self.max_thrust, self.surge_thrust + self.thrust_increment)
            self.get_logger().info('Forward')
        elif key == 's' or key == 'S':
            self.surge_thrust = max(-self.max_thrust, self.surge_thrust - self.thrust_increment)
            self.get_logger().info('Backward')
        elif key == 'a' or key == 'A':
            self.sway_thrust = min(self.max_thrust, self.sway_thrust + self.thrust_increment)
            self.get_logger().info('Left')
        elif key == 'd' or key == 'D':
            self.sway_thrust = max(-self.max_thrust, self.sway_thrust - self.thrust_increment)
            self.get_logger().info('Right')
        elif key == 'e' or key == 'E':
            self.yaw_thrust = min(self.max_thrust, self.yaw_thrust + self.thrust_increment)
            self.get_logger().info('Turn Right')
        elif key == 'r' or key == 'R':
            self.yaw_thrust = max(-self.max_thrust, self.yaw_thrust - self.thrust_increment)
            self.get_logger().info('Turn Left')
        elif key == 'q' or key == 'Q':
            self.heave_thrust = min(self.max_thrust, self.heave_thrust + self.thrust_increment)
            self.get_logger().info('Up')
        elif key == 'z' or key == 'Z':
            self.heave_thrust = max(-self.max_thrust, self.heave_thrust - self.thrust_increment)
            self.get_logger().info('Down')
        elif key == 'x' or key == 'X':
            self.surge_thrust = 0.0
            self.sway_thrust = 0.0
            self.heave_thrust = 0.0
            self.yaw_thrust = 0.0
            self.get_logger().info('Stop All')
        elif key == '\x1b':  # ESC
            self.get_logger().info('Exiting...')
            return False

        # Display current thrust values
        print(f'\rSurge: {self.surge_thrust:+.2f} | Sway: {self.sway_thrust:+.2f} | Yaw: {self.yaw_thrust:+.2f} | Heave: {self.heave_thrust:+.2f}', end='', flush=True)
        return True

    def publish_thrusters(self):
        """Publish thruster setpoints"""
        # Surge thrusters (forward/backward) + Yaw (rotation)
        # For yaw rotation:
        #   - Positive yaw = turn right (port forward, starboard backward)
        #   - Negative yaw = turn left (port backward, starboard forward)
        surge_port_msg = Float64()
        surge_starboard_msg = Float64()
        surge_port_msg.data = self.surge_thrust + self.yaw_thrust
        surge_starboard_msg.data = self.surge_thrust - self.yaw_thrust

        # Sway thruster (left/right)
        sway_msg = Float64()
        sway_msg.data = self.sway_thrust

        # Heave thrusters (up/down)
        heave_bow_msg = Float64()
        heave_stern_msg = Float64()
        heave_bow_msg.data = self.heave_thrust
        heave_stern_msg.data = self.heave_thrust

        # Publish all setpoints
        self.thruster_surge_port_pub.publish(surge_port_msg)
        self.thruster_surge_starboard_pub.publish(surge_starboard_msg)
        self.thruster_sway_pub.publish(sway_msg)
        self.thruster_heave_bow_pub.publish(heave_bow_msg)
        self.thruster_heave_stern_pub.publish(heave_stern_msg)


def main(args=None):
    rclpy.init(args=args)

    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    try:
        # Set terminal to raw mode for key capture
        tty.setraw(sys.stdin.fileno())

        teleop = Girona500Teleop()

        # Main loop
        running = True
        while running and rclpy.ok():
            rclpy.spin_once(teleop, timeout_sec=0.01)
            key = teleop.get_key()
            if key:
                running = teleop.process_key(key)

        teleop.destroy_node()

    except Exception as e:
        print(f'\nError: {e}')

    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


if __name__ == '__main__':
    main()
