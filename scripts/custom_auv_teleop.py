#!/usr/bin/env python3
"""
Custom AUV Keyboard Teleoperation Node
Controls the Custom AUV using keyboard inputs
- CRP Thrusters (Front/Rear counter-rotating propellers)
- 4 Control Fins (Dorsal/Ventral for pitch, Port/Starboard for yaw)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
import select


class CustomAUVTeleop(Node):
    def __init__(self):
        super().__init__('custom_auv_teleop')

        # Publishers for CRP thrusters
        self.thruster_front_pub = self.create_publisher(
            Float64, '/CustomAUV/ThrusterFront/setpoint', 10)
        self.thruster_rear_pub = self.create_publisher(
            Float64, '/CustomAUV/ThrusterRear/setpoint', 10)

        # Publishers for control fin servos
        self.dorsal_fin_pub = self.create_publisher(
            Float64, '/CustomAUV/DorsalFin/setpoint', 10)
        self.ventral_fin_pub = self.create_publisher(
            Float64, '/CustomAUV/VentralFin/setpoint', 10)
        self.port_rudder_pub = self.create_publisher(
            Float64, '/CustomAUV/PortRudder/setpoint', 10)
        self.starboard_rudder_pub = self.create_publisher(
            Float64, '/CustomAUV/StarboardRudder/setpoint', 10)

        # Control parameters
        self.max_thrust = 1.0  # Normalized: -1.0 to 1.0
        self.thrust_increment = 0.05

        self.max_fin_angle = 0.524  # 30 degrees in radians
        self.fin_increment = 0.087  # 5 degrees in radians

        # Current control values
        self.thrust = 0.0
        self.pitch_angle = 0.0  # Dorsal/Ventral fins
        self.yaw_angle = 0.0    # Port/Starboard rudders

        self.get_logger().info('Custom AUV Keyboard Teleoperation Node Started')
        self.print_instructions()

        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_commands)

    def print_instructions(self):
        msg = """
        ============================================
        Custom AUV Keyboard Teleoperation
        ============================================
        Thruster Controls:
          W/S : Increase / Decrease Thrust
          X   : Stop Thrusters

        Fin Controls:
          I/K : Pitch Up / Pitch Down (Dorsal/Ventral)
          J/L : Yaw Left / Yaw Right (Port/Starboard)
          C   : Center All Fins

        Other:
          ESC : Exit

        Current Values will be displayed below
        ============================================
        """
        print(msg)

    def get_key(self):
        """Get keyboard input in non-blocking mode"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def process_key(self, key):
        """Process keyboard input and update control values"""
        # Thruster controls
        if key == 'w' or key == 'W':
            self.thrust = min(self.max_thrust, self.thrust + self.thrust_increment)
            self.get_logger().info('Thrust +')
        elif key == 's' or key == 'S':
            self.thrust = max(-self.max_thrust, self.thrust - self.thrust_increment)
            self.get_logger().info('Thrust -')
        elif key == 'x' or key == 'X':
            self.thrust = 0.0
            self.get_logger().info('Stop Thrusters')

        # Pitch control (Dorsal/Ventral fins)
        elif key == 'i' or key == 'I':
            self.pitch_angle = min(self.max_fin_angle, self.pitch_angle + self.fin_increment)
            self.get_logger().info('Pitch Up')
        elif key == 'k' or key == 'K':
            self.pitch_angle = max(-self.max_fin_angle, self.pitch_angle - self.fin_increment)
            self.get_logger().info('Pitch Down')

        # Yaw control (Port/Starboard rudders)
        elif key == 'j' or key == 'J':
            self.yaw_angle = min(self.max_fin_angle, self.yaw_angle + self.fin_increment)
            self.get_logger().info('Yaw Left')
        elif key == 'l' or key == 'L':
            self.yaw_angle = max(-self.max_fin_angle, self.yaw_angle - self.fin_increment)
            self.get_logger().info('Yaw Right')

        # Center all fins
        elif key == 'c' or key == 'C':
            self.pitch_angle = 0.0
            self.yaw_angle = 0.0
            self.get_logger().info('Center Fins')

        # Exit
        elif key == '\x1b':  # ESC
            self.get_logger().info('Exiting...')
            return False

        # Display current values
        thrust_pct = self.thrust * 100
        pitch_deg = self.pitch_angle * 180 / 3.14159
        yaw_deg = self.yaw_angle * 180 / 3.14159
        print(f'\rThrust: {thrust_pct:+6.1f}% | Pitch: {pitch_deg:+6.1f}° | Yaw: {yaw_deg:+6.1f}°',
              end='', flush=True)
        return True

    def publish_commands(self):
        """Publish thruster and fin servo setpoints"""
        # CRP Thrusters (same thrust for both, counter-rotating cancels torque)
        thrust_msg = Float64()
        thrust_msg.data = self.thrust
        self.thruster_front_pub.publish(thrust_msg)
        self.thruster_rear_pub.publish(thrust_msg)

        # Pitch control fins (Dorsal and Ventral)
        # Positive pitch = nose up (dorsal fin deflects down, ventral fin deflects up)
        dorsal_msg = Float64()
        ventral_msg = Float64()
        dorsal_msg.data = -self.pitch_angle  # Inverted for correct pitch direction
        ventral_msg.data = -self.pitch_angle
        self.dorsal_fin_pub.publish(dorsal_msg)
        self.ventral_fin_pub.publish(ventral_msg)

        # Yaw control rudders (Port and Starboard)
        # Positive yaw = turn left (both rudders deflect to port side)
        port_msg = Float64()
        starboard_msg = Float64()
        port_msg.data = self.yaw_angle
        starboard_msg.data = self.yaw_angle
        self.port_rudder_pub.publish(port_msg)
        self.starboard_rudder_pub.publish(starboard_msg)


def main(args=None):
    rclpy.init(args=args)

    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    try:
        # Set terminal to raw mode for key capture
        tty.setraw(sys.stdin.fileno())

        teleop = CustomAUVTeleop()

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