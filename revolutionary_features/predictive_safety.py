#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float32, Bool, Int32
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import time
import random

class PredictiveSafetySystem(Node):
    """🛡️ Revolutionary Predictive Safety & Hazard Detection"""
    
    def __init__(self):
        super().__init__('predictive_safety_system')
        
        # ═══ HAZARD DETECTION SYSTEM ═══
        self.hazard_types = {
            'crater': {'risk_level': 0.8, 'detection_range': 15.0},
            'steep_slope': {'risk_level': 0.7, 'detection_range': 10.0},
            'loose_regolith': {'risk_level': 0.6, 'detection_range': 8.0},
            'boulder_field': {'risk_level': 0.9, 'detection_range': 20.0},
            'communication_shadow': {'risk_level': 0.4, 'detection_range': 50.0},
            'dust_devil': {'risk_level': 0.8, 'detection_range': 30.0}
        }
        
        # ═══ PREDICTIVE MODELS ═══
        self.prediction_horizon = 10.0  # seconds
        self.risk_threshold = 0.5
        self.safety_margin = 2.0  # meters
        
        # ═══ CURRENT HAZARD STATUS ═══
        self.detected_hazards = []
        self.predicted_hazards = []
        self.current_risk_level = 0.0
        self.safety_mode = 'normal'  # normal, cautious, emergency
        
        # ═══ SENSOR DATA ═══
        self.current_scan = None
        self.current_image = None
        self.current_pose = None
        self.current_velocity = None
        self.terrain_analysis = {'roughness': 0.0, 'slope': 0.0}
        
        # ═══ PREDICTIVE TRACKING ═══
        self.movement_history = []
        self.hazard_history = []
        self.prediction_accuracy = 0.8
        
        # ═══ AI VISION PROCESSING ═══
        self.bridge = CvBridge()
        self.vision_processor = VisionHazardProcessor()
        
        # ═══ SAFETY METRICS ═══
        self.safety_metrics = {
            'hazards_detected': 0,
            'hazards_avoided': 0,
            'false_positives': 0,
            'prediction_accuracy': 0.0,
            'safety_interventions': 0,
            'risk_assessment_score': 0.0
        }
        
        # ═══ ROS SUBSCRIBERS ═══
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/adaptive_slam_pose', self.pose_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/adaptive_slam_map', self.map_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        # ═══ ROS PUBLISHERS ═══
        self.hazard_pub = self.create_publisher(String, '/hazard_alerts', 10)
        self.risk_pub = self.create_publisher(Float32, '/current_risk_level', 10)
        self.safety_cmd_pub = self.create_publisher(Twist, '/safety_cmd_vel', 10)
        self.prediction_pub = self.create_publisher(String, '/hazard_predictions', 10)
        self.safety_status_pub = self.create_publisher(String, '/safety_status', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_brake', 10)
        self.terrain_pub = self.create_publisher(String, '/terrain_analysis', 10)
        
        # ═══ TIMERS ═══
        self.hazard_detection_timer = self.create_timer(0.2, self.hazard_detection_cycle)  # 5Hz detection
        self.prediction_timer = self.create_timer(0.5, self.predictive_analysis_cycle)  # 2Hz prediction
        self.safety_control_timer = self.create_timer(0.1, self.safety_control_cycle)  # 10Hz safety control
        self.terrain_analysis_timer = self.create_timer(1.0, self.terrain_analysis_cycle)  # 1Hz terrain
        
        self.start_time = time.time()
        
        self.get_logger().info("🛡️ PREDICTIVE SAFETY: Revolutionary hazard detection started!")
        self.get_logger().info("⚡ Features: AI vision, Predictive modeling, Real-time intervention, Terrain analysis")
        
    def lidar_callback(self, msg):
        """Process LiDAR data for hazard detection"""
        
        self.current_scan = msg
        
        # Detect obstacles and hazards in LiDAR data
        self.detect_lidar_hazards(msg)
        
    def camera_callback(self, msg):
        """Process camera data for visual hazard detection"""
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # Detect visual hazards
            visual_hazards = self.vision_processor.detect_visual_hazards(cv_image)
            
            # Add to hazard list
            for hazard in visual_hazards:
                self.add_detected_hazard('visual_hazard', hazard['position'], hazard['confidence'])
                
        except Exception as e:
            self.get_logger().warn(f"Camera processing error: {e}")
            
    def pose_callback(self, msg):
        """Track robot pose for predictive analysis"""
        
        self.current_pose = msg.pose
        
        # Add to movement history
        self.movement_history.append({
            'timestamp': time.time(),
            'position': (msg.pose.position.x, msg.pose.position.y),
            'orientation': self.quaternion_to_yaw(msg.pose.orientation)
        })
        
        # Keep only recent history
        if len(self.movement_history) > 50:
            self.movement_history.pop(0)
            
    def velocity_callback(self, msg):
        """Track current velocity for prediction"""
        
        self.current_velocity = msg
        
    def map_callback(self, msg):
        """Analyze map for potential hazards"""
        
        # Analyze occupancy grid for hazardous patterns
        self.analyze_map_hazards(msg)
        
    def imu_callback(self, msg):
        """Process IMU data for terrain analysis"""
        
        # Analyze acceleration patterns for terrain roughness
        accel_magnitude = math.sqrt(
            msg.linear_acceleration.x**2 + 
            msg.linear_acceleration.y**2 + 
            msg.linear_acceleration.z**2
        )
        
        # Update terrain roughness estimate
        self.terrain_analysis['roughness'] = self.terrain_analysis['roughness'] * 0.9 + accel_magnitude * 0.1
        
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def hazard_detection_cycle(self):
        """🔍 MAIN HAZARD DETECTION CYCLE"""
        
        # ═══ CLEAR OLD HAZARDS ═══
        current_time = time.time()
        self.detected_hazards = [h for h in self.detected_hazards 
                                if current_time - h['timestamp'] < 5.0]
        
        # ═══ INJECT DEMONSTRATION HAZARDS ═══
        self.inject_demo_hazards()
        
        # ═══ CALCULATE CURRENT RISK LEVEL ═══
        self.calculate_current_risk()
        
        # ═══ PUBLISH HAZARD ALERTS ═══
        self.publish_hazard_alerts()
        
    def detect_lidar_hazards(self, scan_msg):
        """Detect hazards from LiDAR data"""
        
        if not scan_msg.ranges:
            return
            
        # ═══ DETECT OBSTACLES ═══
        close_obstacles = []
        for i, r in enumerate(scan_msg.ranges):
            if 0.1 < r < 3.0:  # Close obstacles
                angle = scan_msg.angle_min + i * scan_msg.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                close_obstacles.append((x, y, r))
                
        # ═══ ANALYZE OBSTACLE PATTERNS ═══
        if len(close_obstacles) > 5:
            self.add_detected_hazard('boulder_field', (0, 2), 0.8)
            
        # ═══ DETECT GAPS (POTENTIAL CRATERS) ═══
        range_array = np.array(scan_msg.ranges)
        valid_ranges = range_array[(range_array > 0.1) & (range_array < scan_msg.range_max)]
        
        if len(valid_ranges) > 10:
            range_variance = np.var(valid_ranges)
            if range_variance > 100:  # High variance indicates rough terrain
                self.add_detected_hazard('crater', (0, 5), 0.6)
                
    def analyze_map_hazards(self, map_msg):
        """Analyze occupancy grid for hazards"""
        
        if not map_msg.data:
            return
            
        # Convert to numpy array
        width = map_msg.info.width
        height = map_msg.info.height
        map_array = np.array(map_msg.data).reshape((height, width))
        
        # Look for hazardous patterns
        occupied_cells = np.sum(map_array > 50)  # Occupied threshold
        
        if occupied_cells > width * height * 0.3:  # >30% occupied
            self.add_detected_hazard('boulder_field', (0, 0), 0.7)
            
    def inject_demo_hazards(self):
        """Inject demonstration hazards for testing"""
        
        current_time = time.time() - self.start_time
        
        # ═══ TIME-BASED HAZARD INJECTION ═══
        if 15 < current_time < 16 and not any(h['type'] == 'crater' for h in self.detected_hazards):
            self.add_detected_hazard('crater', (10, 5), 0.9)
            self.get_logger().warn("🕳️ HAZARD DETECTED: Large crater ahead!")
            
        if 25 < current_time < 26 and not any(h['type'] == 'steep_slope' for h in self.detected_hazards):
            self.add_detected_hazard('steep_slope', (8, -3), 0.7)
            self.get_logger().warn("📈 HAZARD DETECTED: Steep slope detected!")
            
        if 40 < current_time < 41 and not any(h['type'] == 'dust_devil' for h in self.detected_hazards):
            self.add_detected_hazard('dust_devil', (15, 10), 0.8)
            self.get_logger().warn("🌪️ HAZARD DETECTED: Dust devil approaching!")
            
        # ═══ RANDOM HAZARD INJECTION ═══
        if random.random() < 0.01:  # 1% chance per cycle
            hazard_type = random.choice(list(self.hazard_types.keys()))
            position = (random.uniform(-10, 10), random.uniform(5, 15))
            confidence = random.uniform(0.5, 0.9)
            self.add_detected_hazard(hazard_type, position, confidence)
            
    def add_detected_hazard(self, hazard_type, position, confidence):
        """Add newly detected hazard"""
        
        hazard = {
            'type': hazard_type,
            'position': position,
            'confidence': confidence,
            'timestamp': time.time(),
            'risk_level': self.hazard_types.get(hazard_type, {}).get('risk_level', 0.5),
            'status': 'detected'
        }
        
        self.detected_hazards.append(hazard)
        self.safety_metrics['hazards_detected'] += 1
        
    def calculate_current_risk(self):
        """Calculate current overall risk level"""
        
        if not self.detected_hazards:
            self.current_risk_level = 0.0
            return
            
        # ═══ DISTANCE-WEIGHTED RISK CALCULATION ═══
        total_risk = 0.0
        total_weight = 0.0
        
        for hazard in self.detected_hazards:
            # Calculate distance to hazard
            if self.current_pose:
                dx = hazard['position'][0] - 0  # Relative to robot
                dy = hazard['position'][1] - 0
                distance = math.sqrt(dx**2 + dy**2)
                
                # Risk decreases with distance
                distance_factor = max(0.1, 1.0 / (1.0 + distance * 0.2))
                
                # Weighted risk
                weight = hazard['confidence'] * distance_factor
                risk_contribution = hazard['risk_level'] * weight
                
                total_risk += risk_contribution
                total_weight += weight
                
        # Normalize risk
        if total_weight > 0:
            self.current_risk_level = min(1.0, total_risk / total_weight)
        else:
            self.current_risk_level = 0.0
            
    def predictive_analysis_cycle(self):
        """🔮 PREDICTIVE HAZARD ANALYSIS"""
        
        # ═══ PREDICT FUTURE ROBOT POSITION ═══
        future_positions = self.predict_future_positions()
        
        # ═══ PREDICT HAZARD INTERACTIONS ═══
        predicted_interactions = self.predict_hazard_interactions(future_positions)
        
        # ═══ UPDATE PREDICTED HAZARDS ═══
        self.predicted_hazards = predicted_interactions
        
        # ═══ PUBLISH PREDICTIONS ═══
        self.publish_predictions()
        
    def predict_future_positions(self):
        """Predict robot positions over prediction horizon"""
        
        if not self.current_pose or not self.current_velocity:
            return []
            
        positions = []
        dt = 0.5  # 0.5 second steps
        steps = int(self.prediction_horizon / dt)
        
        # Current state
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        theta = self.quaternion_to_yaw(self.current_pose.orientation)
        
        # Velocity
        v = self.current_velocity.linear.x
        omega = self.current_velocity.angular.z
        
        # Predict future positions using simple motion model
        for i in range(steps):
            t = (i + 1) * dt
            
            # Update position based on current velocity
            x_pred = x + v * math.cos(theta) * t
            y_pred = y + v * math.sin(theta) * t
            theta_pred = theta + omega * t
            
            positions.append({
                'time': t,
                'position': (x_pred, y_pred),
                'orientation': theta_pred
            })
            
        return positions
        
    def predict_hazard_interactions(self, future_positions):
        """Predict interactions with detected hazards"""
        
        interactions = []
        
        for hazard in self.detected_hazards:
            hazard_pos = hazard['position']
            
            for future_pos in future_positions:
                # Calculate distance to hazard
                robot_pos = future_pos['position']
                distance = math.sqrt(
                    (robot_pos[0] - hazard_pos[0])**2 + 
                    (robot_pos[1] - hazard_pos[1])**2
                )
                
                # Check if collision is likely
                if distance < self.safety_margin:
                    interaction = {
                        'hazard': hazard,
                        'predicted_time': future_pos['time'],
                        'predicted_distance': distance,
                        'collision_risk': 1.0 - (distance / self.safety_margin),
                        'severity': hazard['risk_level'] * (1.0 - distance / self.safety_margin)
                    }
                    interactions.append(interaction)
                    
        return interactions
        
    def safety_control_cycle(self):
        """🛡️ REAL-TIME SAFETY CONTROL"""
        
        # ═══ DETERMINE SAFETY MODE ═══
        if self.current_risk_level > 0.8:
            self.safety_mode = 'emergency'
        elif self.current_risk_level > 0.5:
            self.safety_mode = 'cautious'
        else:
            self.safety_mode = 'normal'
            
        # ═══ APPLY SAFETY INTERVENTIONS ═══
        if self.safety_mode == 'emergency':
            self.execute_emergency_intervention()
        elif self.safety_mode == 'cautious':
            self.execute_cautious_intervention()
            
        # ═══ PUBLISH SAFETY STATUS ═══
        self.publish_safety_status()
        
    def execute_emergency_intervention(self):
        """🚨 EMERGENCY SAFETY INTERVENTION"""
        
        self.get_logger().error("🚨 EMERGENCY INTERVENTION: Critical hazard detected!")
        
        # Emergency brake
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        # Stop command
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.safety_cmd_pub.publish(stop_cmd)
        
        self.safety_metrics['safety_interventions'] += 1
        
    def execute_cautious_intervention(self):
        """⚠️ CAUTIOUS SAFETY INTERVENTION"""
        
        # Reduce speed and modify path
        if self.current_velocity:
            safe_cmd = Twist()
            safe_cmd.linear.x = self.current_velocity.linear.x * 0.5  # Half speed
            safe_cmd.angular.z = self.current_velocity.angular.z * 0.3  # Gentle turns
            
            # Add avoidance behavior
            if self.predicted_hazards:
                closest_hazard = min(self.predicted_hazards, 
                                   key=lambda h: h['predicted_distance'])
                
                # Simple avoidance: turn away from hazard
                hazard_pos = closest_hazard['hazard']['position']
                avoidance_turn = 0.2 if hazard_pos[1] > 0 else -0.2
                safe_cmd.angular.z += avoidance_turn
                
            self.safety_cmd_pub.publish(safe_cmd)
            
    def terrain_analysis_cycle(self):
        """🗺️ CONTINUOUS TERRAIN ANALYSIS"""
        
        # ═══ ANALYZE TERRAIN CHARACTERISTICS ═══
        self.analyze_terrain_slope()
        self.analyze_terrain_stability()
        
        # ═══ PUBLISH TERRAIN ANALYSIS ═══
        terrain_status = (
            f"ROUGHNESS:{self.terrain_analysis['roughness']:.2f} | "
            f"SLOPE:{self.terrain_analysis['slope']:.2f} | "
            f"STABILITY:{'STABLE' if self.terrain_analysis['roughness'] < 5.0 else 'UNSTABLE'}"
        )
        
        terrain_msg = String()
        terrain_msg.data = terrain_status
        self.terrain_pub.publish(terrain_msg)
        
    def analyze_terrain_slope(self):
        """Analyze terrain slope from movement patterns"""
        
        if len(self.movement_history) < 3:
            return
            
        # Calculate elevation changes (simplified)
        recent_positions = self.movement_history[-3:]
        
        elevation_changes = []
        for i in range(1, len(recent_positions)):
            # Simulate elevation change based on movement difficulty
            prev_pos = recent_positions[i-1]['position']
            curr_pos = recent_positions[i]['position']
            
            distance = math.sqrt(
                (curr_pos[0] - prev_pos[0])**2 + 
                (curr_pos[1] - prev_pos[1])**2
            )
            
            # Simulate slope based on terrain roughness
            if distance > 0:
                slope_estimate = self.terrain_analysis['roughness'] / distance
                elevation_changes.append(slope_estimate)
                
        if elevation_changes:
            self.terrain_analysis['slope'] = np.mean(elevation_changes)
            
    def analyze_terrain_stability(self):
        """Analyze terrain stability for hazard prediction"""
        
        # Stability analysis based on multiple factors
        roughness_factor = min(1.0, self.terrain_analysis['roughness'] / 10.0)
        slope_factor = min(1.0, abs(self.terrain_analysis['slope']) / 0.5)
        
        instability_score = (roughness_factor + slope_factor) / 2.0
        
        # Generate stability hazard if unstable
        if instability_score > 0.7 and random.random() < 0.1:
            self.add_detected_hazard('loose_regolith', (3, 2), instability_score)
            
    def publish_hazard_alerts(self):
        """📢 PUBLISH HAZARD ALERTS"""
        
        if self.detected_hazards:
            # Create alert message
            alert_parts = []
            for hazard in self.detected_hazards[-3:]:  # Last 3 hazards
                distance = math.sqrt(hazard['position'][0]**2 + hazard['position'][1]**2)
                alert = f"{hazard['type'].upper()}@{distance:.1f}m({hazard['confidence']:.1%})"
                alert_parts.append(alert)
                
            alert_msg = String()
            alert_msg.data = " | ".join(alert_parts)
            self.hazard_pub.publish(alert_msg)
            
        # Publish risk level
        risk_msg = Float32()
        risk_msg.data = self.current_risk_level
        self.risk_pub.publish(risk_msg)
        
    def publish_predictions(self):
        """🔮 PUBLISH HAZARD PREDICTIONS"""
        
        if self.predicted_hazards:
            prediction_parts = []
            for pred in self.predicted_hazards[:2]:  # Top 2 predictions
                hazard_type = pred['hazard']['type']
                time_to_collision = pred['predicted_time']
                risk = pred['collision_risk']
                
                pred_text = f"{hazard_type.upper()}:T+{time_to_collision:.1f}s(RISK:{risk:.1%})"
                prediction_parts.append(pred_text)
                
            prediction_msg = String()
            prediction_msg.data = " | ".join(prediction_parts)
            self.prediction_pub.publish(prediction_msg)
            
    def publish_safety_status(self):
        """📊 PUBLISH SAFETY STATUS"""
        
        status_parts = [
            f"MODE:{self.safety_mode.upper()}",
            f"RISK:{self.current_risk_level:.1%}",
            f"HAZARDS:{len(self.detected_hazards)}",
            f"PREDICTIONS:{len(self.predicted_hazards)}"
        ]
        
        if self.safety_mode == 'emergency':
            status_parts.append("🚨EMERGENCY")
        elif self.safety_mode == 'cautious':
            status_parts.append("⚠️CAUTIOUS")
            
        status_msg = String()
        status_msg.data = " | ".join(status_parts)
        self.safety_status_pub.publish(status_msg)


class VisionHazardProcessor:
    """AI-powered visual hazard detection"""
    
    def __init__(self):
        self.hazard_classifiers = {
            'crater': self.detect_crater,
            'boulder': self.detect_boulder,
            'slope': self.detect_slope
        }
        
    def detect_visual_hazards(self, image):
        """Detect hazards in camera image"""
        
        hazards = []
        
        # Convert to different color spaces for analysis
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Detect different hazard types
        for hazard_type, detector in self.hazard_classifiers.items():
            detected = detector(image, gray, hsv)
            hazards.extend(detected)
            
        return hazards
        
    def detect_crater(self, image, gray, hsv):
        """Detect craters in image"""
        
        # Simple crater detection using circular features
        circles = cv2.HoughCircles(
            gray, cv2.HOUGH_GRADIENT, dp=1, minDist=50,
            param1=50, param2=30, minRadius=20, maxRadius=100
        )
        
        craters = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Convert image coordinates to world coordinates (simplified)
                world_x = (x - image.shape[1]//2) * 0.01  # Rough conversion
                world_y = (image.shape[0] - y) * 0.01
                
                craters.append({
                    'type': 'crater',
                    'position': (world_x, world_y),
                    'confidence': 0.6,
                    'size': r
                })
                
        return craters
        
    def detect_boulder(self, image, gray, hsv):
        """Detect boulders in image"""
        
        # Edge detection for boulder identification
        edges = cv2.Canny(gray, 50, 150)
        
        # Find contours
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        boulders = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            if 100 < area < 5000:  # Filter by size
                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)
                
                # Convert to world coordinates
                world_x = (x + w//2 - image.shape[1]//2) * 0.01
                world_y = (image.shape[0] - (y + h//2)) * 0.01
                
                boulders.append({
                    'type': 'boulder',
                    'position': (world_x, world_y),
                    'confidence': 0.5,
                    'size': area
                })
                
        return boulders
        
    def detect_slope(self, image, gray, hsv):
        """Detect steep slopes in image"""
        
        # Gradient analysis for slope detection
        grad_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
        
        gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
        
        # Find high-gradient regions
        steep_areas = gradient_magnitude > np.percentile(gradient_magnitude, 90)
        
        slopes = []
        if np.any(steep_areas):
            # Find centroid of steep area
            y_coords, x_coords = np.where(steep_areas)
            
            if len(x_coords) > 0:
                center_x = np.mean(x_coords)
                center_y = np.mean(y_coords)
                
                # Convert to world coordinates
                world_x = (center_x - image.shape[1]//2) * 0.01
                world_y = (image.shape[0] - center_y) * 0.01
                
                slopes.append({
                    'type': 'slope',
                    'position': (world_x, world_y),
                    'confidence': 0.4,
                    'severity': np.mean(gradient_magnitude[steep_areas])
                })
                
        return slopes


def main():
    rclpy.init()
    safety_system = PredictiveSafetySystem()
    
    try:
        rclpy.spin(safety_system)
    except KeyboardInterrupt:
        safety_system.get_logger().info("🛡️ Predictive Safety System stopped")
    finally:
        safety_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
