#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class TrajectoryPlannerNode(Node):
    """
    Trajectory planner that publishes desired positions for the centroid.
    Generates a circular trajectory around the origin (0,0).
    """
    
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Publisher for desired trajectory
        self.trajectory_pub = self.create_publisher(Pose, '/desired_trajectory', 10)
        
        # Parameters for circular trajectory
        self.declare_parameter('radius', 2.0)  # Circle radius in meters
        self.declare_parameter('angular_velocity', 0.2)   # rad/s
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        self.radius = self.get_parameter('radius').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        update_rate = self.get_parameter('update_rate').value
        
        # Timer for publishing trajectory
        self.timer = self.create_timer(1.0 / update_rate, self.publish_trajectory)
        
        # State
        self.angle = 0.0
        self.dt = 1.0 / update_rate
        
        self.get_logger().info(f"Trajectory planner started: circular trajectory around origin")
        self.get_logger().info(f"  Radius: {self.radius}m, Angular velocity: {self.angular_velocity} rad/s")
    
    def publish_trajectory(self):
        """Publish circular trajectory around origin"""
        
        pose_msg = Pose()
        
        # Circular trajectory: x = R*cos(theta), y = R*sin(theta)
        pose_msg.position.x = self.radius * math.cos(self.angle)
        pose_msg.position.y = self.radius * math.sin(self.angle)
        pose_msg.position.z = 0.0
        
        # Orientation (not used by controller but for completeness)
        pose_msg.orientation.w = 1.0
        
        self.trajectory_pub.publish(pose_msg)
        
        # Update angle
        self.angle += self.angular_velocity * self.dt
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi
        
        self.get_logger().debug(f"Desired position: ({pose_msg.position.x:.2f}, {pose_msg.position.y:.2f}), angle: {math.degrees(self.angle):.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
