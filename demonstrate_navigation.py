#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class NavigationDemo(Node):
    """Demonstrate autonomous navigation with obstacle avoidance"""
    
    def __init__(self):
        super().__init__('navigation_demo')
        self.cmd_pub = self.create_publisher(Twist, '/lunabot/cmd_vel', 10)
        self.get_logger().info("🚀 Navigation Demo Started!")
        
    def demonstrate_path_planning(self):
        """Demonstrate path planning around obstacles"""
        
        # Path 1: Forward exploration
        self.get_logger().info("🔍 Phase 1: Forward Exploration")
        self.move_forward(2.0)
        
        # Path 2: Obstacle detection and avoidance
        self.get_logger().info("🚧 Phase 2: Obstacle Avoidance")
        self.turn_left(1.0)
        self.move_forward(1.5)
        
        # Path 3: Alternative path
        self.get_logger().info("🔄 Phase 3: Alternative Path")
        self.turn_right(1.5)
        self.move_forward(2.0)
        
        # Path 4: Area coverage pattern
        self.get_logger().info("📊 Phase 4: Area Coverage")
        self.spiral_pattern()
        
        self.get_logger().info("✅ Navigation Demonstration Complete!")
        
    def move_forward(self, duration):
        cmd = Twist()
        cmd.linear.x = 0.3
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
        
        self.stop()
        
    def turn_left(self, duration):
        cmd = Twist()
        cmd.angular.z = 0.5
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
            
        self.stop()
        
    def turn_right(self, duration):
        cmd = Twist()
        cmd.angular.z = -0.5
        
        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.cmd_pub.publish(cmd)
            time.sleep(0.1)
            
        self.stop()
        
    def spiral_pattern(self):
        """Demonstrate spiral exploration pattern"""
        
        for i in range(8):
            # Forward movement (increasing distance)
            self.move_forward(0.5 + i * 0.2)
            # Turn for spiral
            self.turn_left(0.3)
    
    def stop(self):
        cmd = Twist()  # All zeros
        self.cmd_pub.publish(cmd)
        time.sleep(0.5)

def main():
    rclpy.init()
    
    demo = NavigationDemo()
    
    try:
        # Wait a bit for system to be ready
        demo.get_logger().info("⏱️  Starting navigation demo in 3 seconds...")
        time.sleep(3)
        
        # Run demonstration
        demo.demonstrate_path_planning()
        
    except KeyboardInterrupt:
        demo.get_logger().info("🛑 Navigation demo stopped")
        
    finally:
        demo.stop()
        demo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
