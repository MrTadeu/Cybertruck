#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import time

class JoystickMapper(Node):
    def __init__(self):
        super().__init__('joystick_mapper')
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Scaling factors
        self.scale_linear_forward = 1.0
        self.scale_linear_backward = 1.0
        self.scale_angular = 2.0
        self.speed_multiplier = 1.0
        self.speed_increment = 0.1
        self.max_speed_multiplier = 10.0
        self.min_speed_multiplier = 0.5

        # Timestamps for button presses
        self.r1_pressed_time = None
        self.l1_pressed_time = None

    def joy_callback(self, msg: Joy):
        current_time = time.time()  # Current timestamp
        increment = 0.0  # Default value for increment
        decrement = 0.0  # Default value for decrement

        # R1 button pressed (increase speed multiplier)
        if msg.buttons[5]:  # R1 button
            if self.r1_pressed_time is None:  # Button pressed for the first time
                self.r1_pressed_time = current_time
            else:
                # Calculate how long the button has been held
                duration = current_time - self.r1_pressed_time
                increment = self.speed_increment * (2 ** duration)  # Accelerate increment over time
                self.speed_multiplier = min(self.speed_multiplier + increment, self.max_speed_multiplier)
        else:
            self.r1_pressed_time = None  # Reset when button is released

        # L1 button pressed (decrease speed multiplier)
        if msg.buttons[4]:  # L1 button
            if self.l1_pressed_time is None:  # Button pressed for the first time
                self.l1_pressed_time = current_time
            else:
                # Calculate how long the button has been held
                duration = current_time - self.l1_pressed_time
                decrement = self.speed_increment * (2 ** duration)  # Accelerate decrement over time
                self.speed_multiplier = max(self.speed_multiplier - decrement, self.min_speed_multiplier)
        else:
            self.l1_pressed_time = None  # Reset when button is released

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
        self.get_logger().info(f"Speed Multiplier: {self.speed_multiplier:.1f}, Linear Vel: {linear_vel:.2f}, Angular Vel: {angular:.2f}, Speed Increment multiplier: {increment:.2f}, Speed Decrement multiplier: {decrement:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = JoystickMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()