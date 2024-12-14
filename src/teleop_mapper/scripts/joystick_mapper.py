#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray

class JoystickMapper(Node):
    def __init__(self):
        super().__init__('joystick_mapper')
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Scaling factors
        self.scale_linear_forward = 1.0  # Forward throttle
        self.scale_linear_backward = 1.0  # Reverse throttle
        self.scale_angular = 2.0  # Steering
        self.speed_multiplier = 1.0  # Speed multiplier
        self.speed_increment = 0.1  # Speed adjustment step
        self.max_speed_multiplier = 10.0  # Maximum speed multiplier
        self.min_speed_multiplier = 0.5  # Minimum speed multiplier

    def joy_callback(self, msg: Joy):
        # Adjust speed multiplier with R1 (increase) and L1 (decrease)
        if msg.buttons[5]:  # R1 button pressed
            self.speed_multiplier = min(self.speed_multiplier + self.speed_increment, self.max_speed_multiplier)
        if msg.buttons[4]:  # L1 button pressed
            self.speed_multiplier = max(self.speed_multiplier - self.speed_increment, self.min_speed_multiplier)

        # Get trigger and stick inputs
        forward = (msg.axes[2] + 1) / 2 * self.scale_linear_forward * self.speed_multiplier  # L2 normalized
        backward = (msg.axes[5] + 1) / 2 * self.scale_linear_backward * self.speed_multiplier  # R2 normalized
        angular = msg.axes[0] * self.scale_angular  # L3 horizontal

        # Calculate linear velocity
        linear_vel = forward - backward  # Combine forward and backward

        # Publish velocity command
        vel_msg = Float64MultiArray()
        vel_msg.data = [linear_vel, linear_vel, linear_vel, linear_vel]
        self.velocity_publisher.publish(vel_msg)

        # Publish steering command
        steer_msg = Float64MultiArray()
        steer_msg.data = [angular, angular, 0.0, 0.0]
        self.steering_publisher.publish(steer_msg)

        # Debug logs
        self.get_logger().info(f"Speed Multiplier: {self.speed_multiplier:.1f}, Linear Vel: {linear_vel:.2f}, Angular Vel: {angular:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
