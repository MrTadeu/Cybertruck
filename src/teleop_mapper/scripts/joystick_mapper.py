#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray, String
import time

class JoystickMapper(Node):
    def __init__(self):
        super().__init__('joystick_mapper')
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.status_publisher = self.create_publisher(String, '/safety_status', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Scaling factors
        self.scale_linear_forward = 1.0
        self.scale_linear_backward = 1.0
        self.scale_angular = 2.0
        self.speed_multiplier = 1.0
        self.speed_increment = 0.1
        self.max_speed_multiplier = 10.0
        self.min_speed_multiplier = 0.1

        # Safety lock state and toggle
        self.safety_lock = False
        self.prev_x_button_state = False  # Previous state of the X button

    def joy_callback(self, msg: Joy):
        # Toggle safety lock when X button is pressed
        if msg.buttons[0] and not self.prev_x_button_state:  # X button (PS5)
            self.safety_lock = not self.safety_lock
            # Publish safety status
            status_msg = String()
            status_msg.data = "ON" if not self.safety_lock else "OFF"
            self.status_publisher.publish(status_msg)

        # Update previous state of the X button
        self.prev_x_button_state = msg.buttons[0]

        # If safety lock is active, do not publish any commands
        if self.safety_lock:
            self.get_logger().info("Safety lock is active (RED). No commands are being published.")
            return
        else:
            self.get_logger().info("Safety lock is inactive (GREEN). Commands are being published.")

        # R1 button pressed (increase speed multiplier)
        if msg.buttons[5]:  # R1 button
            self.speed_multiplier = min(self.speed_multiplier + self.speed_increment, self.max_speed_multiplier)

        # L1 button pressed (decrease speed multiplier)
        if msg.buttons[4]:  # L1 button
            self.speed_multiplier = max(self.speed_multiplier - self.speed_increment, self.min_speed_multiplier)

        # Get trigger and stick inputs
        forward = (msg.axes[2] + 1) / 2 * self.scale_linear_forward * self.speed_multiplier
        backward = (msg.axes[5] + 1) / 2 * self.scale_linear_backward * self.speed_multiplier
        angular = msg.axes[0] * self.scale_angular

        # Calculate linear velocity
        linear_vel = forward - backward

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
