#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import math

class TrajectoryPlannerNode(Node):
    """
    Simple trajectory planner that publishes desired positions for the centroid.
    Generates circular or straight-line trajectories.
    """
    
    def __init__(self):
        super().__init__('trajectory_planner')
        
        # Publisher for desired trajectory
        self.trajectory_pub = self.create_publisher(Pose, '/desired_trajectory', 10)
        
        # Parameters
        self.declare_parameter('trajectory_type', 'circle')  # 'circle', 'line', 'square', 'static'
        self.declare_parameter('radius', 2.0)  # For circle
        self.declare_parameter('speed', 0.1)   # m/s
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        self.trajectory_type = self.get_parameter('trajectory_type').value
        self.radius = self.get_parameter('radius').value
        self.speed = self.get_parameter('speed').value
        update_rate = self.get_parameter('update_rate').value
        
        # Timer for publishing trajectory
        self.timer = self.create_timer(1.0 / update_rate, self.publish_trajectory)
        
        # State
        self.time = 0.0
        self.dt = 1.0 / update_rate
        
        self.get_logger().info(f"Trajectory planner started: type={self.trajectory_type}, radius={self.radius}, speed={self.speed}")
    
    def publish_trajectory(self):
        """Publish desired position based on trajectory type"""
        
        pose_msg = Pose()
        
        if self.trajectory_type == 'circle':
            # Circular trajectory
            omega = self.speed / self.radius  # Angular velocity
            angle = omega * self.time
            
            pose_msg.position.x = self.radius * math.cos(angle)
            pose_msg.position.y = self.radius * math.sin(angle)
            pose_msg.position.z = 0.0
            
        elif self.trajectory_type == 'line':
            # Straight line along x-axis
            pose_msg.position.x = self.speed * self.time
            pose_msg.position.y = 0.0
            pose_msg.position.z = 0.0
            
        elif self.trajectory_type == 'square':
            # Square trajectory
            perimeter = 4.0 * 2.0 * self.radius  # 4 sides of length 2*radius
            distance = (self.speed * self.time) % perimeter
            
            if distance < 2.0 * self.radius:
                # Side 1: along +x
                pose_msg.position.x = distance
                pose_msg.position.y = 0.0
            elif distance < 4.0 * self.radius:
                # Side 2: along +y
                pose_msg.position.x = 2.0 * self.radius
                pose_msg.position.y = distance - 2.0 * self.radius
            elif distance < 6.0 * self.radius:
                # Side 3: along -x
                pose_msg.position.x = 2.0 * self.radius - (distance - 4.0 * self.radius)
                pose_msg.position.y = 2.0 * self.radius
            else:
                # Side 4: along -y
                pose_msg.position.x = 0.0
                pose_msg.position.y = 2.0 * self.radius - (distance - 6.0 * self.radius)
            
            pose_msg.position.z = 0.0
            
        elif self.trajectory_type == 'static':
            # Static position
            pose_msg.position.x = 2.0
            pose_msg.position.y = 2.0
            pose_msg.position.z = 0.0
        
        else:
            self.get_logger().warn(f"Unknown trajectory type: {self.trajectory_type}")
            return
        
        self.trajectory_pub.publish(pose_msg)
        self.time += self.dt
        
        self.get_logger().debug(f"Published desired position: ({pose_msg.position.x:.2f}, {pose_msg.position.y:.2f})")


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
