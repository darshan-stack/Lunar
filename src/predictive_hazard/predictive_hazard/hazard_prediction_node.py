#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String
import math

class PredictiveHazardNode(Node):
    """AI-powered hazard prediction for lunar rover safety"""
    
    def __init__(self):
        super().__init__('predictive_hazard_node')
        
        # Hazard detection parameters
        self.hazard_threshold = 0.7
        self.crater_min_depth = 0.5
        self.rock_min_height = 0.3
        
        # Publishers
        self.hazard_pub = self.create_publisher(PoseArray, '/hazard_locations', 10)
        self.alert_pub = self.create_publisher(String, '/hazard_alerts', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/lunabot/scan', self.scan_callback, 10)
        
        # Processing timer
        self.create_timer(1.0, self.process_hazards)
        
        self.detected_hazards = []
        self.latest_scan = None
        
        self.get_logger().info("🛡️  Predictive Hazard AI: Lunar safety system active!")
        self.get_logger().info("🔍 Monitoring for craters, rocks, and navigation hazards")
        
    def scan_callback(self, msg):
        self.latest_scan = msg
        
    def process_hazards(self):
        """Process sensor data to predict hazards"""
        
        if self.latest_scan is None:
            return
            
        # Analyze scan data for hazards
        hazards = self.detect_hazards_from_scan(self.latest_scan)
        
        # Update hazard list
        self.detected_hazards = hazards
        
        # Publish hazard locations
        self.publish_hazards(hazards)
        
        # Generate alerts if needed
        self.generate_alerts(hazards)
        
    def detect_hazards_from_scan(self, scan):
        """Detect hazards from LiDAR scan"""
        hazards = []
        ranges = np.array(scan.ranges)
        
        if len(ranges) == 0:
            return hazards
            
        # Detect sudden distance changes (craters/cliffs)
        for i in range(1, len(ranges)-1):
            if ranges[i] > 0.1 and ranges[i] < 30.0:
                
                # Check for crater (sudden increase in distance)
                if ranges[i] > ranges[i-1] * 2.0 and ranges[i] > self.crater_min_depth:
                    angle = scan.angle_min + i * scan.angle_increment
                    hazard_x = ranges[i] * math.cos(angle)
                    hazard_y = ranges[i] * math.sin(angle)
                    
                    hazards.append({
                        'type': 'crater',
                        'x': hazard_x,
                        'y': hazard_y,
                        'severity': min(1.0, ranges[i] / 5.0)
                    })
                    
                # Check for obstacle (sudden decrease in distance)  
                elif ranges[i] < ranges[i-1] * 0.5 and ranges[i] < 2.0:
                    angle = scan.angle_min + i * scan.angle_increment
                    hazard_x = ranges[i] * math.cos(angle)
                    hazard_y = ranges[i] * math.sin(angle)
                    
                    hazards.append({
                        'type': 'obstacle',
                        'x': hazard_x, 
                        'y': hazard_y,
                        'severity': min(1.0, (2.0 - ranges[i]) / 2.0)
                    })
                    
        return hazards
        
    def publish_hazards(self, hazards):
        """Publish detected hazard locations"""
        pose_array = PoseArray()
        pose_array.header.frame_id = "base_link"
        pose_array.header.stamp = self.get_clock().now().to_msg()
        
        for hazard in hazards:
            pose = Pose()
            pose.position.x = hazard['x']
            pose.position.y = hazard['y']
            pose.position.z = 0.0
            
            # Encode hazard type in orientation
            if hazard['type'] == 'crater':
                pose.orientation.z = 0.7071  # 90 degrees
            else:  # obstacle
                pose.orientation.z = 0.0
                
            pose.orientation.w = hazard['severity']
            pose_array.poses.append(pose)
            
        self.hazard_pub.publish(pose_array)
        
    def generate_alerts(self, hazards):
        """Generate safety alerts"""
        
        if not hazards:
            return
            
        high_severity = [h for h in hazards if h['severity'] > self.hazard_threshold]
        
        if high_severity:
            alert_msg = String()
            crater_count = len([h for h in high_severity if h['type'] == 'crater'])
            obstacle_count = len([h for h in high_severity if h['type'] == 'obstacle'])
            
            alert_msg.data = f"⚠️  HIGH RISK: {crater_count} craters, {obstacle_count} obstacles detected"
            self.alert_pub.publish(alert_msg)
            
            self.get_logger().warn(f"🚨 HAZARD ALERT: {len(high_severity)} high-risk hazards detected")
        else:
            # Low risk status
            alert_msg = String()
            alert_msg.data = f"✅ SAFE PATH: {len(hazards)} low-risk features detected"
            self.alert_pub.publish(alert_msg)
            
            self.get_logger().info(f"🛡️  Safe navigation: {len(hazards)} hazards monitored")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        hazard_node = PredictiveHazardNode()
        rclpy.spin(hazard_node)
    except KeyboardInterrupt:
        pass
    finally:
        hazard_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
