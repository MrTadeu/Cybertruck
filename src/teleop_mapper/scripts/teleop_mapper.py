#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class TeleopMapper(Node):
    def __init__(self):
        super().__init__('teleop_mapper')
        self.velocity_publisher = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', 10)
        self.steering_publisher = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg: Twist):
        # Map linear velocity to wheel velocity
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z

        # Adjust steering based on angular velocity
        steering_angle = angular_vel  # Simple mapping, refine as needed

        # Publish velocity command
        vel_msg = Float64MultiArray()
        vel_msg.data = [linear_vel, linear_vel, linear_vel, linear_vel]
        self.velocity_publisher.publish(vel_msg)

        # Publish steering command
        steer_msg = Float64MultiArray()
        steer_msg.data = [steering_angle, steering_angle, 0.0, 0.0]  # Adjust for front steering
        self.steering_publisher.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
