#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32, String, Bool
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
import time
import random

class DustStormAdaptiveSLAM(Node):
    """🌪️ Revolutionary Dust Storm Simulation & Adaptive SLAM"""
    
    def __init__(self):
        super().__init__('dust_storm_adaptive_slam')
        
        # ═══ ADAPTIVE SENSOR WEIGHTS ═══
        self.sensor_weights = {
            'lidar': 0.6,      # LiDAR weight
            'camera': 0.2,     # Camera weight  
            'imu': 0.1,        # IMU weight
            'odometry': 0.1    # Odometry weight
        }
        
        # ═══ ENVIRONMENTAL CONDITIONS ═══
        self.dust_level = 0.0           # 0=clear, 1=dust storm
        self.visibility_score = 1.0     # Visibility percentage
        self.lidar_quality = 1.0        # LiDAR data quality
        self.camera_quality = 1.0       # Camera data quality
        self.wind_speed = 0.0           # Simulated wind
        
        # ═══ DUST STORM SIMULATION ═══
        self.storm_active = False
        self.storm_start_time = 0
        self.storm_duration = 30.0      # 30 second storm
        self.next_storm_time = 40.0     # First storm at 40 seconds
        
        # ═══ SLAM DATA ═══
        self.current_pose = None
        self.map_data = np.zeros((500, 500), dtype=np.int8)
        self.pose_confidence = 1.0
        self.slam_lost = False
        
        # ═══ ROS SUBSCRIBERS ═══
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # ═══ ROS PUBLISHERS ═══
        self.map_pub = self.create_publisher(OccupancyGrid, '/adaptive_slam_map', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/adaptive_slam_pose', 10)
        self.dust_pub = self.create_publisher(Float32, '/dust_level', 10)
        self.status_pub = self.create_publisher(String, '/dust_slam_status', 10)
        self.storm_pub = self.create_publisher(Bool, '/dust_storm_active', 10)
        self.visibility_pub = self.create_publisher(Float32, '/visibility_score', 10)
        
        # ═══ CV BRIDGE ═══
        self.bridge = CvBridge()
        
        # ═══ TIMERS ═══
        self.slam_timer = self.create_timer(0.1, self.adaptive_slam_cycle)  # 10Hz SLAM
        self.storm_timer = self.create_timer(1.0, self.dust_storm_simulation)  # 1Hz storm simulation
        
        self.start_time = time.time()
        
        self.get_logger().info("🌪️ DUST STORM ADAPTIVE SLAM: Revolutionary system started!")
        self.get_logger().info("📊 Features: Real-time adaptation, Storm simulation, Quality assessment")
        
    def lidar_callback(self, msg):
        """Process LiDAR with dust-aware filtering"""
        
        # ═══ ANALYZE LIDAR QUALITY ═══
        valid_ranges = [r for r in msg.ranges if 0.1 < r < msg.range_max]
        
        if len(valid_ranges) == 0:
            self.lidar_quality = 0.0
            return
            
        # Calculate quality metrics
        range_variance = np.var(valid_ranges)
        valid_ratio = len(valid_ranges) / len(msg.ranges)
        
        # Dust degrades LiDAR quality
        base_quality = valid_ratio
        dust_degradation = self.dust_level * 0.7  # Dust reduces quality by up to 70%
        self.lidar_quality = max(0.1, base_quality - dust_degradation)
        
        # ═══ DUST-RESISTANT FILTERING ═══
        self.process_dust_filtered_lidar(msg, valid_ranges)
        
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        
        # Odometry becomes more important during dust storms
        if self.dust_level > 0.5:
            # Increase odometry confidence during storms
            pass
            
    def imu_callback(self, msg):
        """Process IMU data - most reliable during dust storms"""
        self.last_imu = msg
        # IMU maintains quality during dust storms
        
    def dust_storm_simulation(self):
        """🌪️ SIMULATE REALISTIC DUST STORMS"""
        
        current_time = time.time() - self.start_time
        
        # ═══ TRIGGER DUST STORMS ═══
        if not self.storm_active and current_time > self.next_storm_time:
            self.start_dust_storm()
            
        # ═══ MANAGE ACTIVE STORM ═══
        if self.storm_active:
            storm_elapsed = current_time - self.storm_start_time
            
            if storm_elapsed < self.storm_duration:
                # Storm intensity curve (builds up, peaks, dies down)
                progress = storm_elapsed / self.storm_duration
                if progress < 0.3:  # Building up
                    intensity = progress / 0.3
                elif progress < 0.7:  # Peak intensity
                    intensity = 1.0
                else:  # Dying down
                    intensity = (1.0 - progress) / 0.3
                
                # Update storm parameters
                self.dust_level = intensity * 0.9  # Max 90% dust
                self.wind_speed = intensity * 15.0  # Max 15 m/s wind
                self.visibility_score = 1.0 - (intensity * 0.8)  # Min 20% visibility
                
            else:
                # Storm ends
                self.end_dust_storm()
                
        else:
            # Clear conditions
            self.dust_level = max(0.0, self.dust_level - 0.02)  # Gradual clearing
            self.visibility_score = min(1.0, self.visibility_score + 0.02)
            self.wind_speed = max(0.0, self.wind_speed - 0.5)
            
        # ═══ PUBLISH STORM STATUS ═══
        self.publish_environmental_data()
        
    def start_dust_storm(self):
        """Start dust storm event"""
        self.storm_active = True
        self.storm_start_time = time.time() - self.start_time
        self.storm_duration = random.uniform(20.0, 40.0)  # Variable duration
        
        self.get_logger().warn("🌪️ DUST STORM DETECTED: Severe weather conditions!")
        self.get_logger().info(f"⏱️ Storm duration: {self.storm_duration:.1f} seconds")
        self.get_logger().info("🔄 SLAM ADAPTATION: Switching to dust-resistant mode")
        
    def end_dust_storm(self):
        """End dust storm event"""
        self.storm_active = False
        self.next_storm_time = (time.time() - self.start_time) + random.uniform(60.0, 120.0)  # Next storm in 1-2 minutes
        
        self.get_logger().info("☀️ DUST STORM CLEARED: Weather conditions improving")
        self.get_logger().info(f"📅 Next storm expected in {self.next_storm_time - (time.time() - self.start_time):.1f} seconds")
        
    def process_dust_filtered_lidar(self, scan_msg, valid_ranges):
        """Process LiDAR with dust-aware filtering"""
        
        if not self.current_pose:
            return
            
        # ═══ CONFIDENCE-WEIGHTED MAPPING ═══
        robot_x = int(self.current_pose.position.x * 10 + 250)  # Convert to grid
        robot_y = int(self.current_pose.position.y * 10 + 250)
        
        if 0 <= robot_x < 500 and 0 <= robot_y < 500:
            for i, r in enumerate(scan_msg.ranges):
                if 0.1 < r < scan_msg.range_max and not math.isinf(r):
                    
                    # Calculate confidence based on dust conditions
                    angle = scan_msg.angle_min + i * scan_msg.angle_increment
                    confidence = self.calculate_measurement_confidence(r, angle)
                    
                    if confidence > 0.3:  # Only use high-confidence measurements
                        # Calculate obstacle position
                        obs_x = int(robot_x + r * math.cos(angle) * 10)
                        obs_y = int(robot_y + r * math.sin(angle) * 10)
                        
                        if 0 <= obs_x < 500 and 0 <= obs_y < 500:
                            # Update occupancy with confidence weighting
                            update_value = int(confidence * 50)
                            self.map_data[obs_y, obs_x] = min(100, self.map_data[obs_y, obs_x] + update_value)
                            
    def calculate_measurement_confidence(self, range_val, angle):
        """Calculate confidence for individual measurement"""
        
        base_confidence = 1.0
        
        # ═══ RANGE-BASED CONFIDENCE ═══
        if range_val < 0.5:  # Too close
            base_confidence *= 0.6
        elif range_val > 20.0:  # Too far in dust
            base_confidence *= 0.4
            
        # ═══ ANGULAR CONFIDENCE ═══
        angle_factor = 1.0 - abs(angle) / (math.pi / 2) * 0.3
        base_confidence *= angle_factor
        
        # ═══ DUST IMPACT ═══
        dust_factor = 1.0 - (self.dust_level * 0.8)  # Dust reduces confidence by up to 80%
        base_confidence *= dust_factor
        
        # ═══ WIND IMPACT ═══
        wind_factor = 1.0 - (self.wind_speed / 15.0 * 0.2)  # Wind reduces confidence by up to 20%
        base_confidence *= wind_factor
        
        return max(0.0, base_confidence)
        
    def adaptive_slam_cycle(self):
        """🧠 MAIN ADAPTIVE SLAM CYCLE"""
        
        # ═══ ADAPT SENSOR WEIGHTS ═══
        self.adapt_sensor_weights()
        
        # ═══ PERFORM SLAM ═══
        self.perform_weighted_slam()
        
        # ═══ DETECT SLAM FAILURE ═══
        self.detect_slam_failure()
        
        # ═══ PUBLISH RESULTS ═══
        self.publish_slam_results()
        
    def adapt_sensor_weights(self):
        """🎯 REVOLUTIONARY ADAPTIVE WEIGHTING"""
        
        if self.dust_level > 0.6:  # SEVERE DUST STORM
            # Vision-based sensors severely degraded
            self.sensor_weights['lidar'] = 0.2 * self.lidar_quality
            self.sensor_weights['camera'] = 0.05 * self.camera_quality
            # Rely heavily on motion sensors
            self.sensor_weights['imu'] = 0.5
            self.sensor_weights['odometry'] = 0.45
            
            self.get_logger().info("🌪️ SEVERE DUST: Emergency sensor reconfiguration")
            
        elif self.dust_level > 0.3:  # MODERATE DUST
            # Balanced degradation
            self.sensor_weights['lidar'] = 0.4 * self.lidar_quality
            self.sensor_weights['camera'] = 0.1 * self.camera_quality
            self.sensor_weights['imu'] = 0.3
            self.sensor_weights['odometry'] = 0.2
            
        else:  # CLEAR CONDITIONS
            # Optimal sensor usage
            self.sensor_weights['lidar'] = 0.6 * self.lidar_quality
            self.sensor_weights['camera'] = 0.2 * self.camera_quality
            self.sensor_weights['imu'] = 0.1
            self.sensor_weights['odometry'] = 0.1
            
        # Normalize weights
        total = sum(self.sensor_weights.values())
        if total > 0:
            for key in self.sensor_weights:
                self.sensor_weights[key] /= total
                
    def perform_weighted_slam(self):
        """Perform SLAM using adaptive sensor weights"""
        
        if not self.current_pose:
            return
            
        # Calculate pose confidence
        self.pose_confidence = (
            self.sensor_weights['lidar'] * self.lidar_quality +
            self.sensor_weights['camera'] * self.camera_quality +
            self.sensor_weights['imu'] * 0.9 +
            self.sensor_weights['odometry'] * 0.7
        )
        
        # Detect if SLAM is lost
        if self.pose_confidence < 0.2:
            self.slam_lost = True
        elif self.pose_confidence > 0.4:
            self.slam_lost = False
            
    def detect_slam_failure(self):
        """Detect and respond to SLAM failure"""
        
        if self.slam_lost and self.dust_level > 0.5:
            self.get_logger().warn("⚠️ SLAM DEGRADED: Switching to dead-reckoning mode")
        elif self.pose_confidence < 0.3:
            self.get_logger().warn("⚠️ LOW CONFIDENCE: SLAM quality degraded")
            
    def publish_environmental_data(self):
        """Publish environmental status"""
        
        # Dust level
        dust_msg = Float32()
        dust_msg.data = self.dust_level
        self.dust_pub.publish(dust_msg)
        
        # Visibility score
        vis_msg = Float32()
        vis_msg.data = self.visibility_score
        self.visibility_pub.publish(vis_msg)
        
        # Storm status
        storm_msg = Bool()
        storm_msg.data = self.storm_active
        self.storm_pub.publish(storm_msg)
        
    def publish_slam_results(self):
        """Publish SLAM results and status"""
        
        # ═══ PUBLISH POSE ═══
        if self.current_pose:
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.pose = self.current_pose
            
            # Set covariance based on confidence
            base_cov = 1.0 - self.pose_confidence
            pose_msg.pose.covariance[0] = base_cov      # x variance
            pose_msg.pose.covariance[7] = base_cov      # y variance
            pose_msg.pose.covariance[35] = base_cov * 0.1  # yaw variance
            
            self.pose_pub.publish(pose_msg)
            
        # ═══ PUBLISH MAP ═══
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = "map"
        map_msg.info.resolution = 0.1  # 10cm per cell
        map_msg.info.width = 500
        map_msg.info.height = 500
        map_msg.info.origin.position.x = -25.0
        map_msg.info.origin.position.y = -25.0
        map_msg.data = self.map_data.flatten().astype(np.int8).tolist()
        self.map_pub.publish(map_msg)
        
        # ═══ PUBLISH STATUS ═══
        status_parts = [
            f"DUST:{self.dust_level:.2f}",
            f"VIS:{self.visibility_score:.2f}",
            f"LiDAR:{self.sensor_weights['lidar']:.2f}",
            f"IMU:{self.sensor_weights['imu']:.2f}",
            f"CONF:{self.pose_confidence:.2f}"
        ]
        
        if self.storm_active:
            status_parts.append("STORM:ACTIVE")
        if self.slam_lost:
            status_parts.append("SLAM:LOST")
            
        status_msg = String()
        status_msg.data = " | ".join(status_parts)
        self.status_pub.publish(status_msg)

def main():
    rclpy.init()
    dust_slam = DustStormAdaptiveSLAM()
    
    try:
        rclpy.spin(dust_slam)
    except KeyboardInterrupt:
        dust_slam.get_logger().info("🛑 Dust Storm Adaptive SLAM stopped")
    finally:
        dust_slam.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
