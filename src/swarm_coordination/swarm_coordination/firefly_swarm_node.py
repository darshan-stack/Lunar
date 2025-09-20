#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Float32MultiArray
import threading
import time

class FireflySwarmCoordinator(Node):
    """Firefly-inspired multi-robot coordination for lunar exploration"""
    
    def __init__(self):
        super().__init__('firefly_swarm_coordinator')
        
        # Swarm parameters
        self.num_robots = 3
        self.robot_poses = {}
        self.robot_objectives = {}
        self.sync_pattern = np.zeros(self.num_robots)
        
        # Publishers for each robot
        self.cmd_publishers = {}
        for i in range(self.num_robots):
            self.cmd_publishers[f'robot_{i}'] = self.create_publisher(
                Twist, f'/robot_{i}/cmd_vel', 10
            )
            
        # Swarm coordination publisher
        self.swarm_pub = self.create_publisher(String, '/swarm_status', 10)
        self.pattern_pub = self.create_publisher(Float32MultiArray, '/firefly_pattern', 10)
        
        # Coordination timer
        self.create_timer(0.5, self.coordinate_swarm)  # 2Hz coordination
        
        # Initialize robot positions
        self.initialize_swarm()
        
        self.get_logger().info("🦋 Firefly Swarm Coordinator: Multi-robot coordination active!")
        self.get_logger().info("🌙 Coordinating 3 lunar rovers for optimal exploration coverage")
        
    def initialize_swarm(self):
        """Initialize swarm in formation"""
        formations = [
            {'x': 0, 'y': 0, 'objective': 'mapping'},
            {'x': 5, 'y': 5, 'objective': 'scouting'}, 
            {'x': -5, 'y': 5, 'objective': 'backup'}
        ]
        
        for i, formation in enumerate(formations):
            self.robot_poses[f'robot_{i}'] = formation
            self.robot_objectives[f'robot_{i}'] = formation['objective']
            
    def coordinate_swarm(self):
        """Main swarm coordination using firefly synchronization"""
        
        # Update firefly synchronization pattern
        self.update_firefly_pattern()
        
        # Coordinate robots based on pattern
        for i in range(self.num_robots):
            robot_id = f'robot_{i}'
            cmd = self.calculate_robot_command(robot_id, i)
            self.cmd_publishers[robot_id].publish(cmd)
            
        # Publish swarm status
        self.publish_swarm_status()
        
    def update_firefly_pattern(self):
        """Update firefly synchronization pattern"""
        current_time = time.time()
        
        for i in range(self.num_robots):
            # Firefly oscillation with phase coupling
            phase = (current_time + i * 2.0) * 0.5
            self.sync_pattern[i] = math.sin(phase) * 0.5 + 0.5
            
            # Add synchronization influence from neighbors
            for j in range(self.num_robots):
                if i != j:
                    influence = 0.1 * math.sin(self.sync_pattern[j] * math.pi)
                    self.sync_pattern[i] += influence
                    
        # Normalize pattern
        self.sync_pattern = np.clip(self.sync_pattern, 0, 1)
        
    def calculate_robot_command(self, robot_id, robot_idx):
        """Calculate movement command for robot based on swarm behavior"""
        cmd = Twist()
        
        # Base movement pattern
        sync_value = self.sync_pattern[robot_idx]
        
        if self.robot_objectives[robot_id] == 'mapping':
            # Main robot: steady exploration
            cmd.linear.x = 0.3 * sync_value
            cmd.angular.z = 0.1 * math.sin(sync_value * math.pi)
            
        elif self.robot_objectives[robot_id] == 'scouting':
            # Scout robot: wide area coverage
            cmd.linear.x = 0.4 * sync_value
            cmd.angular.z = 0.3 * math.cos(sync_value * math.pi)
            
        else:  # backup
            # Backup robot: conservative following
            cmd.linear.x = 0.2 * sync_value
            cmd.angular.z = 0.05 * math.sin(sync_value * math.pi * 0.5)
            
        return cmd
        
    def publish_swarm_status(self):
        """Publish swarm coordination status"""
        
        # Status message
        status_msg = String()
        sync_level = np.mean(self.sync_pattern)
        
        if sync_level > 0.7:
            status_msg.data = f"🦋 SWARM SYNCHRONIZED: {sync_level:.2f} - Optimal coordination"
        elif sync_level > 0.4:
            status_msg.data = f"🔄 SWARM ADAPTING: {sync_level:.2f} - Moderate coordination"
        else:
            status_msg.data = f"⚡ SWARM INITIALIZING: {sync_level:.2f} - Building coordination"
            
        self.swarm_pub.publish(status_msg)
        
        # Pattern data
        pattern_msg = Float32MultiArray()
        pattern_msg.data = self.sync_pattern.tolist()
        self.pattern_pub.publish(pattern_msg)
        
        # Log coordination
        self.get_logger().info(f"🦋 Firefly Sync: {sync_level:.2f} | Robots coordinated")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        swarm_coordinator = FireflySwarmCoordinator()
        rclpy.spin(swarm_coordinator)
    except KeyboardInterrupt:
        pass
    finally:
        swarm_coordinator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
