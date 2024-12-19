#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TeleopMapper(Node):
    def __init__(self):
        super().__init__('teleop_mapper')
        # Publishers for velocity and steering commands
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        # Subscriber to Twist messages
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        linear_vel = max(-10.0, min(10.0, linear_vel))

        angular_vel = max(-2.0, min(2.0, angular_vel))

        # Publish the scaled and clamped velocity command
        vel_msg = Float64MultiArray()
        vel_msg.data = [linear_vel, linear_vel, linear_vel, linear_vel]
        self.velocity_publisher.publish(vel_msg)

        # Publish the scaled and clamped steering command
        steer_msg = Float64MultiArray()
        steer_msg.data = [angular_vel, angular_vel, 0.0, 0.0]
        self.steering_publisher.publish(steer_msg)

        # Log the processed values
        self.get_logger().info(f"Mapped Linear Vel: {linear_vel:.2f}, Mapped Angular Vel: {angular_vel:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
