#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
import time
import threading
from collections import deque

# ROS2 message types
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray, Header, String

class SimpleEnvironmentalAssessor:
    """Lightweight environmental condition assessment"""
    
    def __init__(self):
        self.dust_factor = 0.0
        self.visibility = 1.0
        self.sensor_weights = {
            'odometry': 0.6,
            'lidar': 0.3,
            'imu': 0.1
        }
        
    def update_conditions(self, odom_data, scan_data=None):
        """Update environmental assessment"""
        
        if scan_data is not None:
            # Assess dust from scan quality
            valid_ranges = [r for r in scan_data if 0.1 < r < 30.0]
            if len(valid_ranges) > 10:
                range_variance = np.var(valid_ranges)
                self.dust_factor = min(1.0, range_variance * 0.1)
            else:
                self.dust_factor = 0.8  # High dust if few valid readings
                
        # Adjust weights based on conditions
        if self.dust_factor > 0.5:
            # High dust - rely more on odometry
            self.sensor_weights = {
                'odometry': 0.8,
                'lidar': 0.15,
                'imu': 0.05
            }
        else:
            # Clear conditions - use more LiDAR
            self.sensor_weights = {
                'odometry': 0.4,
                'lidar': 0.5,
                'imu': 0.1
            }
            
    def get_weights(self):
        return self.sensor_weights

class SimpleSLAMCore:
    """Lightweight SLAM implementation"""
    
    def __init__(self):
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.pose_history = deque(maxlen=100)
        
        # Simple occupancy grid
        self.map_size = 200
        self.map_resolution = 0.1
        self.occupancy_grid = np.full((self.map_size, self.map_size), -1, dtype=np.int8)
        
    def update_pose(self, odom_msg, scan_ranges=None, weights=None):
        """Update robot pose estimate"""
        
        if odom_msg is None:
            return
            
        # Extract pose from odometry
        pos = odom_msg.pose.pose.position
        ori = odom_msg.pose.pose.orientation
        
        # Simple quaternion to euler conversion
        siny_cosp = 2 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1 - 2 * (ori.y * ori.y + ori.z * ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update pose (in future this would include sensor fusion)
        self.robot_pose = [pos.x, pos.y, yaw]
        self.pose_history.append(self.robot_pose.copy())
        
        # Update map if we have scan data
        if scan_ranges is not None:
            self.update_map(scan_ranges)
            
    def update_map(self, scan_ranges):
        """Update occupancy grid with scan data"""
        
        if not scan_ranges or len(scan_ranges) == 0:
            return
            
        # Get robot position in map coordinates
        map_x = int(self.robot_pose[0] / self.map_resolution) + self.map_size // 2
        map_y = int(self.robot_pose[1] / self.map_resolution) + self.map_size // 2
        
        if 0 <= map_x < self.map_size and 0 <= map_y < self.map_size:
            # Mark robot position as free
            self.occupancy_grid[map_y, map_x] = 0
            
            # Process scan data
            num_beams = len(scan_ranges)
            angle_increment = 2 * math.pi / num_beams
            
            for i, distance in enumerate(scan_ranges):
                if 0.1 < distance < 30.0:  # Valid range
                    beam_angle = self.robot_pose[2] + (i * angle_increment) - math.pi
                    
                    # Calculate obstacle position
                    obs_x = self.robot_pose[0] + distance * math.cos(beam_angle)
                    obs_y = self.robot_pose[1] + distance * math.sin(beam_angle)
                    
                    # Convert to map coordinates
                    obs_map_x = int(obs_x / self.map_resolution) + self.map_size // 2
                    obs_map_y = int(obs_y / self.map_resolution) + self.map_size // 2
                    
                    # Mark as occupied
                    if 0 <= obs_map_x < self.map_size and 0 <= obs_map_y < self.map_size:
                        self.occupancy_grid[obs_map_y, obs_map_x] = 100
                        
    def get_map_msg(self):
        """Convert map to ROS OccupancyGrid message"""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = rclpy.time.Time().to_msg()
        
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_size
        grid_msg.info.height = self.map_size
        
        # Map origin
        grid_msg.info.origin.position.x = -self.map_size * self.map_resolution / 2.0
        grid_msg.info.origin.position.y = -self.map_size * self.map_resolution / 2.0
        grid_msg.info.origin.position.z = 0.0
        
        # Convert to ROS format
        grid_msg.data = self.occupancy_grid.flatten().astype(np.int8).tolist()
        
        return grid_msg

class SimpleAMFSSLAMNode(Node):
    """Simplified Adaptive SLAM Node"""
    
    def __init__(self):
        super().__init__('simple_adaptive_slam_node')
        
        # Initialize components
        self.env_assessor = SimpleEnvironmentalAssessor()
        self.slam_core = SimpleSLAMCore()
        
        # Data storage
        self.latest_odom = None
        self.latest_scan = None
        self.last_update_time = time.time()
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'slam_map', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'slam_pose', 10)
        self.weights_pub = self.create_publisher(Float32MultiArray, 'sensor_weights', 10)
        self.status_pub = self.create_publisher(String, 'slam_status', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/lunabot/odom', self.odom_callback, 10)
        
        # Try to subscribe to scan if available
        self.scan_sub = self.create_subscription(LaserScan, '/lunabot/scan', self.scan_callback, 10)
        
        # Main processing timer
        self.create_timer(0.2, self.slam_update_loop)  # 5 Hz
        
        # Status reporting timer
        self.create_timer(2.0, self.publish_status)  # Every 2 seconds
        
        self.get_logger().info("🚀 Simple AMFS-SLAM: Started successfully!")
        self.get_logger().info("🌙 Adaptive Multi-Sensor Fusion SLAM for lunar environments")
        self.get_logger().info("📡 Subscribing to /lunabot/odom and /lunabot/scan")
        
    def odom_callback(self, msg):
        self.latest_odom = msg
        
    def scan_callback(self, msg):
        self.latest_scan = msg.ranges
        
    def slam_update_loop(self):
        """Main SLAM processing loop"""
        
        if self.latest_odom is None:
            return
            
        # Update environmental conditions
        scan_data = list(self.latest_scan) if self.latest_scan else None
        self.env_assessor.update_conditions(self.latest_odom, scan_data)
        
        # Get current sensor weights
        weights = self.env_assessor.get_weights()
        
        # Update SLAM
        self.slam_core.update_pose(self.latest_odom, scan_data, weights)
        
        # Publish results
        self.publish_results(weights)
        
    def publish_results(self, weights):
        """Publish SLAM results"""
        
        # Publish map
        map_msg = self.slam_core.get_map_msg()
        self.map_pub.publish(map_msg)
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.slam_core.robot_pose[0]
        pose_msg.pose.position.y = self.slam_core.robot_pose[1]
        pose_msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion (simplified)
        yaw = self.slam_core.robot_pose[2]
        pose_msg.pose.orientation.w = math.cos(yaw / 2)
        pose_msg.pose.orientation.z = math.sin(yaw / 2)
        
        self.pose_pub.publish(pose_msg)
        
        # Publish sensor weights
        weights_msg = Float32MultiArray()
        weights_msg.data = [
            weights['odometry'],
            weights['lidar'],
            weights['imu']
        ]
        self.weights_pub.publish(weights_msg)
        
        # Log adaptation
        dust_level = self.env_assessor.dust_factor
        if dust_level > 0.5:
            self.get_logger().info(f"🌪️  HIGH DUST: Odometry weight increased to {weights['odometry']:.2f}")
        elif dust_level > 0.3:
            self.get_logger().info(f"🌫️  MODERATE DUST: Balanced sensor fusion")
        else:
            self.get_logger().info(f"✨ CLEAR CONDITIONS: LiDAR weight {weights['lidar']:.2f}")
            
    def publish_status(self):
        """Publish system status"""
        status_msg = String()
        
        scan_status = "ACTIVE" if self.latest_scan else "NO_SCAN"
        odom_status = "ACTIVE" if self.latest_odom else "NO_ODOM"
        
        dust_level = "HIGH" if self.env_assessor.dust_factor > 0.5 else "LOW"
        
        status_msg.data = f"AMFS-SLAM: Scan={scan_status}, Odom={odom_status}, Dust={dust_level}"
        self.status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        slam_node = SimpleAMFSSLAMNode()
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'slam_node' in locals():
            slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
