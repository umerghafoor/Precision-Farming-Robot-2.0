#!/usr/bin/env python3
"""
ROS2 Motor Controller Node for SPI Communication
Subscribes to /cmd_vel and sends 6D motor control signals via SPI
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import spidev
import time


class MotorControllerNode(Node):
    """ROS2 node for controlling motors via SPI based on cmd_vel commands"""
    
    # Motor direction constants
    DIR_FORWARD = 0
    DIR_BACKWARD = 1
    DIR_STOP = 2
    
    def __init__(self):
        """Initialize ROS2 node with SPI communication"""
        super().__init__('motor_controller_node')
        
        # Banner
        self.get_logger().info("=" * 60)
        self.get_logger().info("    6D Signal Motor Controller - SPI Master (ROS2)")
        self.get_logger().info("=" * 60)
        
        # Declare and get parameters
        self._declare_parameters()
        self._load_parameters()
        
        # Initialize SPI
        self._init_spi()
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Log startup info
        self.get_logger().info("✓ Motor Controller Node initialized")
        self.get_logger().info(f"✓ Subscribed to /cmd_vel topic")
        self.get_logger().info(f"✓ Robot parameters:")
        self.get_logger().info(f"    - Wheel base: {self.wheel_base}m")
        self.get_logger().info(f"    - Wheel radius: {self.wheel_radius}m")
        self.get_logger().info(f"    - Max speed: {self.max_speed}")
        self.get_logger().info(f"    - Max linear vel: {self.max_linear_vel} m/s")
        self.get_logger().info(f"    - Max angular vel: {self.max_angular_vel} rad/s")
        self.get_logger().info("=" * 60)
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters"""
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('max_speed', 255)
        self.declare_parameter('max_linear_vel', 1.0)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('spi_bus', 0)
        self.declare_parameter('spi_device', 0)
        self.declare_parameter('spi_speed_hz', 500000)
        self.declare_parameter('spi_mode', 0)
    
    def _load_parameters(self):
        """Load parameters from ROS2 parameter server"""
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.spi_bus = self.get_parameter('spi_bus').value
        self.spi_device = self.get_parameter('spi_device').value
        self.spi_speed_hz = self.get_parameter('spi_speed_hz').value
        self.spi_mode = self.get_parameter('spi_mode').value
    
    def _init_spi(self):
        """Initialize SPI communication"""
        try:
            self.spi = spidev.SpiDev()
            self.spi.open(self.spi_bus, self.spi_device)
            self.spi.max_speed_hz = self.spi_speed_hz
            self.spi.mode = self.spi_mode
            self.get_logger().info(f"✓ SPI initialized: Bus {self.spi_bus}, Device {self.spi_device}")
            self.get_logger().info(f"✓ SPI Speed: {self.spi_speed_hz} Hz, Mode: {self.spi_mode}")
        except Exception as e:
            self.get_logger().error(f"✗ Failed to initialize SPI: {e}")
            self.get_logger().error("  Make sure:")
            self.get_logger().error("    1. SPI is enabled (sudo raspi-config)")
            self.get_logger().error("    2. User has SPI permissions")
            self.get_logger().error("    3. /dev/spidev0.0 exists")
            raise
    
    def cmd_vel_callback(self, msg):
        """
        Callback for /cmd_vel topic - converts Twist to motor commands
        
        Args:
            msg (Twist): Velocity command message
        """
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        self.get_logger().info(
            f"cmd_vel: linear=[{linear_x:.2f}, {linear_y:.2f}], angular={angular_z:.2f}",
            throttle_duration_sec=1.0
        )
        
        # Convert to motor commands
        motor_commands = self.twist_to_motor_commands(linear_x, linear_y, angular_z)
        
        # Send via SPI
        self.send_6d_signal(*motor_commands)
    
    def twist_to_motor_commands(self, linear_x, linear_y, angular_z):
        """
        Convert twist velocities to 6D motor commands
        
        For holonomic 3-wheel robot:
        - Motor 1: Front-Left (120°)
        - Motor 2: Front-Right (240°)
        - Motor 3: Back (0°)
        
        Args:
            linear_x: Forward/backward velocity (m/s)
            linear_y: Left/right velocity (m/s)
            angular_z: Rotation velocity (rad/s)
            
        Returns:
            tuple: (motor1_dir, motor1_speed, motor2_dir, motor2_speed, motor3_dir, motor3_speed)
        """
        # Clamp input velocities
        linear_x = self._clamp(linear_x, -self.max_linear_vel, self.max_linear_vel)
        linear_y = self._clamp(linear_y, -self.max_linear_vel, self.max_linear_vel)
        angular_z = self._clamp(angular_z, -self.max_angular_vel, self.max_angular_vel)
        
        # For differential drive (2-wheel) robot:
        if abs(linear_y) < 0.01:  # Standard differential drive
            left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
            right_vel = linear_x + (angular_z * self.wheel_base / 2.0)
            
            motor1_speed, motor1_dir = self._vel_to_motor(left_vel)
            motor2_speed, motor2_dir = self._vel_to_motor(right_vel)
            motor3_speed, motor3_dir = 0, self.DIR_STOP
        
        else:  # Holonomic/omni-directional (3-wheel) robot
            # Inverse kinematics for 3-wheel omni robot
            # Motor angles: 120°, 240°, 0° from front
            import math
            
            # Motor 1 (120°): -0.5*vx - 0.866*vy + w*r
            v1 = -0.5 * linear_x - 0.866 * linear_y + angular_z * self.wheel_base
            
            # Motor 2 (240°): -0.5*vx + 0.866*vy + w*r
            v2 = -0.5 * linear_x + 0.866 * linear_y + angular_z * self.wheel_base
            
            # Motor 3 (0°): 1.0*vx + 0*vy + w*r
            v3 = linear_x + angular_z * self.wheel_base
            
            motor1_speed, motor1_dir = self._vel_to_motor(v1)
            motor2_speed, motor2_dir = self._vel_to_motor(v2)
            motor3_speed, motor3_dir = self._vel_to_motor(v3)
        
        return (motor1_dir, motor1_speed, motor2_dir, motor2_speed, motor3_dir, motor3_speed)
    
    def _vel_to_motor(self, velocity):
        """
        Convert velocity to motor speed and direction
        
        Args:
            velocity: Motor velocity (m/s or normalized)
            
        Returns:
            tuple: (speed, direction)
        """
        if abs(velocity) < 0.01:
            return 0, self.DIR_STOP
        
        # Normalize and scale to motor speed
        speed = int(abs(velocity / self.max_linear_vel) * self.max_speed)
        speed = self._clamp(speed, 0, self.max_speed)
        
        # Determine direction
        direction = self.DIR_FORWARD if velocity > 0 else self.DIR_BACKWARD
        
        return speed, direction
    
    def _clamp(self, value, min_val, max_val):
        """Clamp value between min and max"""
        return max(min_val, min(max_val, value))
    
    def send_6d_signal(self, motor1_dir, motor1_speed, motor2_dir, motor2_speed, motor3_dir, motor3_speed):
        """
        Send 6D signal packet via SPI
        
        Packet format (6 bytes):
        [motor1_dir, motor1_speed, motor2_dir, motor2_speed, motor3_dir, motor3_speed]
        
        Args:
            motor1_dir: Direction for motor 1 (0=Forward, 1=Backward, 2=Stop)
            motor1_speed: Speed for motor 1 (0-255)
            motor2_dir: Direction for motor 2
            motor2_speed: Speed for motor 2 (0-255)
            motor3_dir: Direction for motor 3
            motor3_speed: Speed for motor 3 (0-255)
            
        Returns:
            bool: True if successful, False otherwise
        """
        # Build 6-byte packet
        packet = [
            motor1_dir & 0xFF,
            motor1_speed & 0xFF,
            motor2_dir & 0xFF,
            motor2_speed & 0xFF,
            motor3_dir & 0xFF,
            motor3_speed & 0xFF
        ]
        
        # Log packet details
        self.get_logger().debug("--- Sending 6D Signal ---")
        self.get_logger().debug(f"M1: {self._dir_to_str(motor1_dir):8s} Speed={motor1_speed:3d}")
        self.get_logger().debug(f"M2: {self._dir_to_str(motor2_dir):8s} Speed={motor2_speed:3d}")
        self.get_logger().debug(f"M3: {self._dir_to_str(motor3_dir):8s} Speed={motor3_speed:3d}")
        self.get_logger().debug(f"RAW: [{' '.join(f'0x{b:02X}' for b in packet)}]")
        
        try:
            # Transfer data via SPI
            response = self.spi.xfer2(packet)
            self.get_logger().debug(f"SPI Response: {response}")
            return True
        except Exception as e:
            self.get_logger().error(f"SPI transfer failed: {e}")
            return False
    
    def _dir_to_str(self, direction):
        """Convert direction code to string"""
        mapping = {
            self.DIR_FORWARD: "Forward",
            self.DIR_BACKWARD: "Backward",
            self.DIR_STOP: "Stop"
        }
        return mapping.get(direction, "Unknown")
    
    def destroy_node(self):
        """Cleanup before node destruction"""
        try:
            if hasattr(self, 'spi') and self.spi:
                # Send stop command
                self.send_6d_signal(
                    self.DIR_STOP, 0,
                    self.DIR_STOP, 0,
                    self.DIR_STOP, 0
                )
                time.sleep(0.1)
                
                # Close SPI
                self.spi.close()
                self.get_logger().info("✓ SPI connection closed")
        except Exception as e:
            self.get_logger().error(f"Error during cleanup: {e}")
        
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    node = None
    
    try:
        node = MotorControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutdown requested by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
