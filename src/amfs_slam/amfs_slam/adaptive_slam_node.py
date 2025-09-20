#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from collections import deque
import threading
import time

# ROS2 message types
from sensor_msgs.msg import LaserScan, Image, Imu
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Float32MultiArray, Header
from cv_bridge import CvBridge

# Scientific libraries
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize
import sklearn.cluster as cluster

class EnvironmentalConditionAssessor:
    """Assesses lunar environmental conditions for sensor weighting"""
    
    def __init__(self):
        self.dust_density = 0.0
        self.visibility_factor = 1.0
        self.temperature = -20.0  # Lunar surface temperature
        self.lighting_condition = 1.0
        self.sensor_degradation = {
            'lidar': 0.0,
            'camera': 0.0,
            'imu': 0.0,
            'odometry': 0.0
        }
        
    def update_environmental_conditions(self, lidar_data, image_data, imu_data):
        """Update environmental assessment from sensor data"""
        
        # Assess dust density from LiDAR noise
        if lidar_data is not None:
            self._assess_dust_from_lidar(lidar_data)
            
        # Assess visibility from image quality
        if image_data is not None:
            self._assess_visibility_from_image(image_data)
            
        # Assess vibration/instability from IMU
        if imu_data is not None:
            self._assess_stability_from_imu(imu_data)
            
        # Calculate sensor degradation factors
        self._calculate_sensor_degradation()
        
    def _assess_dust_from_lidar(self, ranges):
        """Assess dust density from LiDAR data quality"""
        # Higher noise = more dust interference
        valid_ranges = ranges[(ranges > 0.1) & (ranges < 30.0)]
        if len(valid_ranges) > 10:
            # Calculate noise level
            range_std = np.std(valid_ranges)
            range_dropouts = np.sum(ranges == 0) / len(ranges)
            
            # Dust factor (0=clear, 1=heavy dust)
            self.dust_density = min(1.0, range_std * 0.1 + range_dropouts * 2.0)
        
    def _assess_visibility_from_image(self, image):
        """Assess visibility from image contrast and sharpness"""
        if image is not None and len(image.shape) >= 2:
            # Convert to grayscale if needed
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image
                
            # Calculate image quality metrics
            contrast = np.std(gray)
            laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # Visibility factor (1=clear, 0=poor visibility)
            self.visibility_factor = min(1.0, (contrast / 50.0) * (laplacian_var / 500.0))
            
    def _assess_stability_from_imu(self, imu_msg):
        """Assess platform stability from IMU vibrations"""
        # Calculate acceleration magnitude changes
        accel_mag = np.sqrt(
            imu_msg.linear_acceleration.x**2 + 
            imu_msg.linear_acceleration.y**2 + 
            imu_msg.linear_acceleration.z**2
        )
        
        # Simple stability assessment (more complex in real implementation)
        vibration_level = abs(accel_mag - 1.62)  # Deviation from lunar gravity
        self.sensor_degradation['imu'] = min(1.0, vibration_level * 0.1)
        
    def _calculate_sensor_degradation(self):
        """Calculate degradation factors for each sensor type"""
        # LiDAR degradation from dust
        self.sensor_degradation['lidar'] = self.dust_density * 0.8
        
        # Camera degradation from visibility
        self.sensor_degradation['camera'] = (1.0 - self.visibility_factor) * 0.9
        
        # Odometry affected by surface conditions
        self.sensor_degradation['odometry'] = (self.dust_density * 0.3)
        
    def get_sensor_weights(self):
        """Get current adaptive sensor weights"""
        # Base weights
        weights = {
            'lidar': 0.4,
            'camera': 0.3,
            'imu': 0.2,
            'odometry': 0.1
        }
        
        # Adjust weights based on environmental conditions
        for sensor in weights:
            degradation = self.sensor_degradation.get(sensor, 0.0)
            # Reduce weight based on degradation
            weights[sensor] *= (1.0 - degradation)
            
        # Normalize weights to sum to 1.0
        total_weight = sum(weights.values())
        if total_weight > 0:
            for sensor in weights:
                weights[sensor] /= total_weight
                
        return weights

class AdaptiveSLAMCore:
    """Core SLAM algorithm with dynamic sensor fusion"""
    
    def __init__(self):
        self.map_resolution = 0.1  # meters per pixel
        self.map_size = (400, 400)  # 40m x 40m map
        self.occupancy_grid = np.full(self.map_size, -1, dtype=np.int8)  # -1=unknown, 0=free, 100=occupied
        
        # Robot pose (x, y, theta)
        self.robot_pose = np.array([0.0, 0.0, 0.0])
        self.pose_covariance = np.eye(3) * 0.1
        
        # Landmark storage
        self.landmarks = []
        self.landmark_covariances = []
        
        # Particle filter for pose tracking
        self.num_particles = 100
        self.particles = np.random.normal([0, 0, 0], [1, 1, 0.1], (self.num_particles, 3))
        self.particle_weights = np.ones(self.num_particles) / self.num_particles
        
        # Feature tracking
        self.previous_features = []
        self.feature_tracker = cv2.TrackerKCF_create() if hasattr(cv2, 'TrackerKCF_create') else None
        
    def update_pose_estimate(self, odometry_delta, lidar_scan, visual_features, sensor_weights):
        """Update robot pose using weighted sensor fusion"""
        
        # 1. Prediction step using odometry
        self._predict_pose(odometry_delta, sensor_weights['odometry'])
        
        # 2. LiDAR-based correction
        if lidar_scan is not None and sensor_weights['lidar'] > 0.1:
            self._lidar_pose_correction(lidar_scan, sensor_weights['lidar'])
            
        # 3. Visual odometry correction
        if visual_features is not None and sensor_weights['camera'] > 0.1:
            self._visual_pose_correction(visual_features, sensor_weights['camera'])
            
        # 4. Update particle filter
        self._update_particle_filter()
        
        # 5. Extract best pose estimate
        self._extract_pose_estimate()
        
    def _predict_pose(self, odometry_delta, odometry_weight):
        """Predict new pose using odometry data"""
        # Add odometry motion to all particles
        for i in range(self.num_particles):
            # Add motion with noise proportional to inverse of weight
            noise_factor = 1.0 / max(0.1, odometry_weight)
            
            dx = odometry_delta[0] + np.random.normal(0, 0.05 * noise_factor)
            dy = odometry_delta[1] + np.random.normal(0, 0.05 * noise_factor)
            dtheta = odometry_delta[2] + np.random.normal(0, 0.02 * noise_factor)
            
            # Update particle pose
            self.particles[i, 0] += dx * np.cos(self.particles[i, 2]) - dy * np.sin(self.particles[i, 2])
            self.particles[i, 1] += dx * np.sin(self.particles[i, 2]) + dy * np.cos(self.particles[i, 2])
            self.particles[i, 2] += dtheta
            
    def _lidar_pose_correction(self, ranges, lidar_weight):
        """Correct pose estimate using LiDAR scan matching"""
        
        # Extract features from LiDAR scan
        lidar_features = self._extract_lidar_features(ranges)
        
        # Match features with existing map
        for i, particle in enumerate(self.particles):
            # Calculate likelihood of this particle given LiDAR data
            likelihood = self._calculate_lidar_likelihood(particle, lidar_features)
            
            # Update particle weight
            self.particle_weights[i] *= (1.0 + likelihood * lidar_weight)
            
    def _visual_pose_correction(self, features, camera_weight):
        """Correct pose using visual features"""
        
        # Track features between frames
        if len(self.previous_features) > 0 and len(features) > 0:
            # Calculate visual motion estimate
            visual_motion = self._estimate_visual_motion(self.previous_features, features)
            
            # Update particle weights based on visual consistency
            for i, particle in enumerate(self.particles):
                # Check if visual motion is consistent with particle motion
                consistency = self._check_visual_consistency(particle, visual_motion)
                self.particle_weights[i] *= (1.0 + consistency * camera_weight)
                
        self.previous_features = features
        
    def _extract_lidar_features(self, ranges):
        """Extract corner and line features from LiDAR scan"""
        features = []
        
        # Convert ranges to cartesian points
        angles = np.linspace(-np.pi, np.pi, len(ranges))
        valid_indices = (ranges > 0.1) & (ranges < 30.0)
        
        valid_ranges = ranges[valid_indices]
        valid_angles = angles[valid_indices]
        
        if len(valid_ranges) > 10:
            # Convert to cartesian
            x_points = valid_ranges * np.cos(valid_angles)
            y_points = valid_ranges * np.sin(valid_angles)
            
            # Simple corner detection using angle changes
            for i in range(1, len(valid_ranges) - 1):
                angle_change = abs(valid_angles[i+1] - valid_angles[i-1])
                if angle_change > 0.3:  # Significant angle change indicates corner
                    features.append({
                        'type': 'corner',
                        'position': np.array([x_points[i], y_points[i]]),
                        'angle': valid_angles[i]
                    })
                    
        return features
        
    def _calculate_lidar_likelihood(self, particle, features):
        """Calculate likelihood of particle pose given LiDAR features"""
        likelihood = 0.0
        
        for feature in features:
            # Transform feature to global coordinates
            global_pos = self._transform_to_global(feature['position'], particle)
            
            # Check if feature matches existing landmarks
            match_found = False
            for landmark in self.landmarks:
                distance = np.linalg.norm(global_pos - landmark['position'])
                if distance < 0.5:  # Within 50cm
                    likelihood += 1.0
                    match_found = True
                    break
                    
            # Penalty for unmatched features (unless map is still being built)
            if not match_found and len(self.landmarks) > 10:
                likelihood -= 0.2
                
        return likelihood
        
    def _transform_to_global(self, local_point, pose):
        """Transform point from robot frame to global frame"""
        x, y, theta = pose
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        global_x = x + local_point[0] * cos_theta - local_point[1] * sin_theta
        global_y = y + local_point[0] * sin_theta + local_point[1] * cos_theta
        
        return np.array([global_x, global_y])
        
    def _estimate_visual_motion(self, prev_features, curr_features):
        """Estimate robot motion from visual feature tracking"""
        # Simplified visual odometry
        if len(prev_features) > 4 and len(curr_features) > 4:
            # Use first few features for motion estimation
            motion_estimates = []
            
            for i in range(min(5, len(prev_features), len(curr_features))):
                dx = curr_features[i][0] - prev_features[i][0]
                dy = curr_features[i][1] - prev_features[i][1]
                motion_estimates.append([dx, dy])
                
            if motion_estimates:
                avg_motion = np.mean(motion_estimates, axis=0)
                return {'dx': avg_motion[0] * 0.01, 'dy': avg_motion[1] * 0.01, 'dtheta': 0.0}
                
        return {'dx': 0.0, 'dy': 0.0, 'dtheta': 0.0}
        
    def _check_visual_consistency(self, particle, visual_motion):
        """Check consistency between particle motion and visual motion"""
        # Simplified consistency check
        predicted_motion = visual_motion
        
        # Calculate difference between predicted and actual motion
        dx_diff = abs(predicted_motion['dx'])
        dy_diff = abs(predicted_motion['dy'])
        
        # Return consistency score (higher = more consistent)
        consistency = max(0.0, 1.0 - (dx_diff + dy_diff) * 10.0)
        return consistency
        
    def _update_particle_filter(self):
        """Update particle filter weights and resample"""
        
        # Normalize weights
        self.particle_weights /= np.sum(self.particle_weights)
        
        # Resample particles if effective sample size is low
        effective_sample_size = 1.0 / np.sum(self.particle_weights**2)
        
        if effective_sample_size < self.num_particles * 0.5:
            # Resample particles
            indices = np.random.choice(
                self.num_particles, 
                self.num_particles, 
                p=self.particle_weights
            )
            
            self.particles = self.particles[indices]
            self.particle_weights = np.ones(self.num_particles) / self.num_particles
            
    def _extract_pose_estimate(self):
        """Extract best pose estimate from particle filter"""
        # Weighted average of particles
        self.robot_pose = np.average(self.particles, weights=self.particle_weights, axis=0)
        
        # Calculate covariance
        weighted_diff = self.particles - self.robot_pose
        self.pose_covariance = np.cov(weighted_diff.T, aweights=self.particle_weights)
        
    def update_map(self, lidar_scan, robot_pose):
        """Update occupancy grid map with new LiDAR scan"""
        
        if lidar_scan is None or len(lidar_scan) == 0:
            return
            
        # Get robot position in map coordinates
        map_x = int((robot_pose[0] / self.map_resolution) + self.map_size[0] // 2)
        map_y = int((robot_pose[1] / self.map_resolution) + self.map_size[1] // 2)
        
        if 0 <= map_x < self.map_size[0] and 0 <= map_y < self.map_size[1]:
            # Mark robot position as free
            self.occupancy_grid[map_y, map_x] = 0
            
            # Process LiDAR scan
            angles = np.linspace(-np.pi, np.pi, len(lidar_scan))
            
            for i, (distance, angle) in enumerate(zip(lidar_scan, angles)):
                if 0.1 < distance < 30.0:  # Valid range
                    # Calculate endpoint in global coordinates
                    global_angle = robot_pose[2] + angle
                    end_x = robot_pose[0] + distance * np.cos(global_angle)
                    end_y = robot_pose[1] + distance * np.sin(global_angle)
                    
                    # Convert to map coordinates
                    end_map_x = int((end_x / self.map_resolution) + self.map_size[0] // 2)
                    end_map_y = int((end_y / self.map_resolution) + self.map_size[1] // 2)
                    
                    # Bresenham's line algorithm to mark free space
                    points = self._bresenham_line(map_x, map_y, end_map_x, end_map_y)
                    
                    for px, py in points[:-1]:  # All points except last are free
                        if 0 <= px < self.map_size[0] and 0 <= py < self.map_size[1]:
                            self.occupancy_grid[py, px] = 0
                            
                    # Mark endpoint as occupied
                    if 0 <= end_map_x < self.map_size[0] and 0 <= end_map_y < self.map_size[1]:
                        self.occupancy_grid[end_map_y, end_map_x] = 100
                        
    def _bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm for ray tracing"""
        points = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        while True:
            points.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return points
        
    def get_map_as_ros_msg(self):
        """Convert occupancy grid to ROS OccupancyGrid message"""
        grid_msg = OccupancyGrid()
        grid_msg.header.frame_id = "map"
        grid_msg.header.stamp = rclpy.time.Time().to_msg()
        
        # Map metadata
        grid_msg.info.resolution = self.map_resolution
        grid_msg.info.width = self.map_size[0]
        grid_msg.info.height = self.map_size[1]
        
        # Map origin (center of map at 0,0)
        grid_msg.info.origin.position.x = -self.map_size[0] * self.map_resolution / 2.0
        grid_msg.info.origin.position.y = -self.map_size[1] * self.map_resolution / 2.0
        grid_msg.info.origin.position.z = 0.0
        
        # Convert occupancy grid to ROS format
        grid_msg.data = self.occupancy_grid.flatten().astype(np.int8).tolist()
        
        return grid_msg

class AdaptiveSLAMNode(Node):
    """Main ROS2 node for Adaptive Multi-Sensor Fusion SLAM"""
    
    def __init__(self):
        super().__init__('adaptive_slam_node')
        
        # Initialize core components
        self.env_assessor = EnvironmentalConditionAssessor()
        self.slam_core = AdaptiveSLAMCore()
        self.bridge = CvBridge()
        
        # Data storage
        self.latest_scan = None
        self.latest_image = None
        self.latest_imu = None
        self.latest_odom = None
        self.previous_odom = None
        
        # Processing control
        self.processing_lock = threading.Lock()
        self.last_update_time = time.time()
        
        # Publishers
        self.map_pub = self.create_publisher(OccupancyGrid, 'slam_map', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'slam_pose', 10)
        self.weights_pub = self.create_publisher(Float32MultiArray, 'sensor_weights', 10)
        
        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # Main processing timer
        self.create_timer(0.1, self.main_slam_loop)  # 10 Hz
        
        self.get_logger().info("🧠 AMFS-SLAM: Adaptive Multi-Sensor Fusion SLAM initialized!")
        self.get_logger().info("🌙 Ready for autonomous lunar navigation with adaptive sensor fusion!")
        
    def scan_callback(self, msg):
        with self.processing_lock:
            self.latest_scan = np.array(msg.ranges)
            
    def image_callback(self, msg):
        try:
            with self.processing_lock:
                cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                self.latest_image = cv_image
        except Exception as e:
            self.get_logger().warn(f"Image conversion failed: {e}")
            
    def imu_callback(self, msg):
        with self.processing_lock:
            self.latest_imu = msg
            
    def odom_callback(self, msg):
        with self.processing_lock:
            self.previous_odom = self.latest_odom
            self.latest_odom = msg
            
    def main_slam_loop(self):
        """Main SLAM processing loop"""
        current_time = time.time()
        
        # Only process if we have minimum required data
        if self.latest_scan is None or self.latest_odom is None:
            return
            
        with self.processing_lock:
            # 1. Update environmental conditions
            self.env_assessor.update_environmental_conditions(
                self.latest_scan, 
                self.latest_image, 
                self.latest_imu
            )
            
            # 2. Get adaptive sensor weights
            sensor_weights = self.env_assessor.get_sensor_weights()
            
            # 3. Calculate odometry delta
            odom_delta = self._calculate_odometry_delta()
            
            # 4. Extract visual features
            visual_features = self._extract_visual_features()
            
            # 5. Update SLAM pose estimate
            self.slam_core.update_pose_estimate(
                odom_delta, 
                self.latest_scan, 
                visual_features, 
                sensor_weights
            )
            
            # 6. Update map
            self.slam_core.update_map(self.latest_scan, self.slam_core.robot_pose)
            
            # 7. Publish results
            self._publish_results(sensor_weights)
            
        # Log performance
        processing_time = time.time() - current_time
        if processing_time > 0.05:  # Warn if processing takes too long
            self.get_logger().warn(f"SLAM processing took {processing_time:.3f}s")
            
        self.last_update_time = current_time
        
    def _calculate_odometry_delta(self):
        """Calculate change in odometry since last update"""
        if self.previous_odom is None:
            return np.array([0.0, 0.0, 0.0])
            
        # Extract poses
        curr_pos = self.latest_odom.pose.pose.position
        curr_rot = self.latest_odom.pose.pose.orientation
        prev_pos = self.previous_odom.pose.pose.position
        prev_rot = self.previous_odom.pose.pose.orientation
        
        # Calculate position change
        dx = curr_pos.x - prev_pos.x
        dy = curr_pos.y - prev_pos.y
        
        # Calculate rotation change
        curr_euler = Rotation.from_quat([curr_rot.x, curr_rot.y, curr_rot.z, curr_rot.w]).as_euler('xyz')
        prev_euler = Rotation.from_quat([prev_rot.x, prev_rot.y, prev_rot.z, prev_rot.w]).as_euler('xyz')
        dtheta = curr_euler[2] - prev_euler[2]
        
        return np.array([dx, dy, dtheta])
        
    def _extract_visual_features(self):
        """Extract visual features from camera image"""
        if self.latest_image is None:
            return None
            
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            
            # Extract corners using FAST detector
            fast = cv2.FastFeatureDetector_create()
            keypoints = fast.detect(gray, None)
            
            # Convert keypoints to simple coordinate list
            features = [(kp.pt[0], kp.pt[1]) for kp in keypoints[:50]]  # Limit to 50 features
            
            return features
            
        except Exception as e:
            self.get_logger().warn(f"Feature extraction failed: {e}")
            return None
            
    def _publish_results(self, sensor_weights):
        """Publish SLAM results"""
        
        # Publish map
        map_msg = self.slam_core.get_map_as_ros_msg()
        self.map_pub.publish(map_msg)
        
        # Publish pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = self.slam_core.robot_pose[0]
        pose_msg.pose.position.y = self.slam_core.robot_pose[1]
        pose_msg.pose.position.z = 0.0
        
        # Convert yaw to quaternion
        yaw = self.slam_core.robot_pose[2]
        quat = Rotation.from_euler('z', yaw).as_quat()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        self.pose_pub.publish(pose_msg)
        
        # Publish sensor weights
        weights_msg = Float32MultiArray()
        weights_msg.data = [
            sensor_weights['lidar'],
            sensor_weights['camera'], 
            sensor_weights['imu'],
            sensor_weights['odometry']
        ]
        self.weights_pub.publish(weights_msg)
        
        # Log adaptive behavior
        if max(sensor_weights.values()) - min(sensor_weights.values()) > 0.2:
            dominant_sensor = max(sensor_weights, key=sensor_weights.get)
            self.get_logger().info(f"🔄 ADAPTIVE: {dominant_sensor} dominant (weight: {sensor_weights[dominant_sensor]:.2f})")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        slam_node = AdaptiveSLAMNode()
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        pass
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
