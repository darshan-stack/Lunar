#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import time

class SimpleRoverController(Node):
    def __init__(self):
        super().__init__('simple_rover_controller')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/rover_status', 10)
        
        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        # State
        self.current_goal = None
        self.current_pose = None
        self.moving = False
        self.goal_start_time = 0
        self.last_scan = None
        
        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("🤖 Simple Rover Controller: Ready for commands!")
        
    def goal_callback(self, msg):
        self.current_goal = (msg.pose.position.x, msg.pose.position.y)
        self.moving = True
        self.goal_start_time = time.time()
        self.get_logger().info(f"🎯 New goal: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})")
        
    def odom_callback(self, msg):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        
    def scan_callback(self, msg):
        self.last_scan = msg
        
    def control_loop(self):
        cmd = Twist()
        
        if self.current_goal and self.moving and self.current_pose:
            # Calculate distance to goal
            dx = self.current_goal[0] - self.current_pose[0]
            dy = self.current_goal[1] - self.current_pose[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            # Check if goal reached
            if distance < 1.0:  # Within 1 meter
                self.moving = False
                self.current_goal = None
                self.get_logger().info("✅ Goal reached!")
            else:
                # Simple navigation toward goal
                target_angle = math.atan2(dy, dx)
                
                # Simple obstacle avoidance
                obstacle_detected = False
                if self.last_scan:
                    # Check front ranges
                    front_ranges = self.last_scan.ranges[160:200]  # Front 40 degrees
                    min_distance = min([r for r in front_ranges if r > 0.1])
                    
                    if min_distance < 2.0:  # Obstacle within 2 meters
                        obstacle_detected = True
                
                if obstacle_detected:
                    # Turn to avoid obstacle
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.5
                    self.get_logger().info("🚧 Avoiding obstacle")
                else:
                    # Move toward goal
                    cmd.linear.x = min(0.5, distance * 0.5)  # Proportional speed
                    cmd.angular.z = target_angle * 0.3  # Simple steering
                    
            # Timeout protection
            if time.time() - self.goal_start_time > 30.0:  # 30 second timeout
                self.moving = False
                self.current_goal = None
                self.get_logger().info("⏰ Goal timeout - stopping")
                
        else:
            # No goal - stay stopped
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            
        self.cmd_pub.publish(cmd)
        
        # Publish status
        if self.current_pose:
            pos_str = f"({self.current_pose[0]:.1f}, {self.current_pose[1]:.1f})"
        else:
            pos_str = "Unknown"
            
        status = f"STATUS: {'MOVING' if self.moving else 'IDLE'} | POS: {pos_str} | GOAL: {'Yes' if self.current_goal else 'No'}"
        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

def main():
    rclpy.init()
    controller = SimpleRoverController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Controller stopped")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
