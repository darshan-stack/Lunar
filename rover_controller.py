#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import threading
import time
import math

class RoverController(Node):
    """Control rover and monitor all systems"""
    
    def __init__(self):
        super().__init__('rover_controller')
        
        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/lunabot/cmd_vel', 10)
        
        # Subscribers for monitoring
        self.odom_sub = self.create_subscription(Odometry, '/lunabot/odom', self.odom_callback, 10)
        self.slam_sub = self.create_subscription(PoseStamped, '/slam_pose', self.slam_callback, 10)
        self.swarm_sub = self.create_subscription(String, '/swarm_status', self.swarm_callback, 10)
        self.hazard_sub = self.create_subscription(String, '/hazard_alerts', self.hazard_callback, 10)
        
        # Current state
        self.current_pose = None
        self.slam_pose = None
        self.swarm_status = "Initializing"
        self.hazard_status = "Monitoring"
        
        # Control thread
        self.control_thread = threading.Thread(target=self.control_loop)
        self.control_thread.daemon = True
        self.control_thread.start()
        
        self.get_logger().info("🎮 ROVER CONTROLLER ACTIVE!")
        self.get_logger().info("🚀 Use WASD keys for manual control, or watch autonomous demo")
        
    def odom_callback(self, msg):
        """Track rover odometry"""
        self.current_pose = msg.pose.pose
        
    def slam_callback(self, msg):
        """Track SLAM pose"""
        self.slam_pose = msg.pose
        
    def swarm_callback(self, msg):
        """Track swarm status"""
        self.swarm_status = msg.data
        
    def hazard_callback(self, msg):
        """Track hazard status"""
        self.hazard_status = msg.data
        
    def control_loop(self):
        """Main control and monitoring loop"""
        
        demo_phase = 0
        last_status_time = time.time()
        
        while rclpy.ok():
            
            # Print status every 3 seconds
            if time.time() - last_status_time > 3.0:
                self.print_system_status()
                last_status_time = time.time()
            
            # Run autonomous demo
            self.run_demo_phase(demo_phase)
            demo_phase = (demo_phase + 1) % 8  # 8 different phases
            
            time.sleep(2.0)  # 2 second intervals
            
    def print_system_status(self):
        """Print comprehensive system status"""
        
        self.get_logger().info("=" * 80)
        self.get_logger().info("🤖 ROVER SYSTEM STATUS")
        self.get_logger().info("=" * 80)
        
        if self.current_pose:
            x = self.current_pose.position.x
            y = self.current_pose.position.y
            z = self.current_pose.position.z
            self.get_logger().info(f"📍 POSITION: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
        else:
            self.get_logger().info("📍 POSITION: Waiting for odometry...")
            
        if self.slam_pose:
            slam_x = self.slam_pose.position.x
            slam_y = self.slam_pose.position.y
            self.get_logger().info(f"🧠 SLAM POSE: X={slam_x:.2f}m, Y={slam_y:.2f}m")
        else:
            self.get_logger().info("🧠 SLAM POSE: Initializing...")
            
        self.get_logger().info(f"🦋 SWARM: {self.swarm_status}")
        self.get_logger().info(f"🛡️  HAZARD: {self.hazard_status}")
        self.get_logger().info("=" * 80)
        
    def run_demo_phase(self, phase):
        """Run different demo phases"""
        
        cmd = Twist()
        
        if phase == 0:
            # Forward exploration
            self.get_logger().info("🔍 PHASE: Forward Exploration")
            cmd.linear.x = 0.5
            
        elif phase == 1:
            # Turn left
            self.get_logger().info("↰ PHASE: Turn Left")
            cmd.angular.z = 0.8
            
        elif phase == 2:
            # Forward again
            self.get_logger().info("⬆️ PHASE: Forward Movement")
            cmd.linear.x = 0.4
            
        elif phase == 3:
            # Turn right
            self.get_logger().info("↱ PHASE: Turn Right")
            cmd.angular.z = -0.8
            
        elif phase == 4:
            # Diagonal movement
            self.get_logger().info("↗️ PHASE: Diagonal Movement")
            cmd.linear.x = 0.3
            cmd.angular.z = 0.2
            
        elif phase == 5:
            # Reverse
            self.get_logger().info("⬇️ PHASE: Reverse Movement")
            cmd.linear.x = -0.3
            
        elif phase == 6:
            # Spin in place
            self.get_logger().info("🔄 PHASE: Spin Maneuver")
            cmd.angular.z = 1.0
            
        else:
            # Stop and assess
            self.get_logger().info("🛑 PHASE: Stop and Assess")
            # cmd remains zero
            
        # Publish command
        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    
    controller = RoverController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Rover controller stopped")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
