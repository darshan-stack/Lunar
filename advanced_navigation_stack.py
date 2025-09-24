#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, Image, Imu
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from std_msgs.msg import String, Float32, Bool
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import cv2
from cv_bridge import CvBridge
import math
import time
from sklearn.cluster import DBSCAN
import heapq
from collections import defaultdict

class AdvancedNavigationStack(Node):
    """🧠 Advanced Navigation Stack with Multi-Sensor Fusion"""
    
    def __init__(self):
        super().__init__('advanced_navigation_stack')
        
        # ═══ MULTI-SENSOR FUSION STATE ═══
        self.lidar_data = None
        self.depth_data = None
        self.rgb_data = None
        self.imu_data = None
        self.current_pose = None
        
        # ═══ OBSTACLE DETECTION ═══
        self.obstacles = []
        self.dynamic_obstacles = []
        self.bridge = CvBridge()
        
        # ═══ COSTMAP & PLANNING ═══
        self.global_costmap = np.zeros((1000, 1000), dtype=np.uint8)
        self.local_costmap = np.zeros((200, 200), dtype=np.uint8)
        self.current_path = []
        self.goal_pose = None
        
        # ═══ SLOPE ANALYSIS ═══
        self.slope_threshold = 0.3  # 30 degrees max
        self.slope_map = np.zeros((1000, 1000), dtype=np.float32)
        
        # ═══ ENERGY OPTIMIZATION ═══
        self.energy_costmap = np.zeros((1000, 1000), dtype=np.float32)
        self.battery_level = 100.0
        
        # ═══ AUTO-RECOVERY STATE ═══
        self.stuck_counter = 0
        self.last_positions = []
        self.recovery_mode = False
        
        # ═══ ROS SUBSCRIBERS ═══
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.depth_sub = self.create_subscription(PointCloud2, '/camera/depth/points', self.depth_callback, 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)
        
        # ═══ ROS PUBLISHERS ═══
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/navigation_path', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/fusion_costmap', 10)
        self.obstacles_pub = self.create_publisher(String, '/detected_obstacles', 10)
        self.nav_status_pub = self.create_publisher(String, '/navigation_status', 10)
        self.slope_pub = self.create_publisher(String, '/slope_analysis', 10)
        self.energy_pub = self.create_publisher(String, '/energy_navigation', 10)
        self.recovery_pub = self.create_publisher(Bool, '/recovery_mode', 10)
        
        # ═══ TIMERS ═══
        self.fusion_timer = self.create_timer(0.1, self.sensor_fusion_cycle)  # 10Hz
        self.planning_timer = self.create_timer(0.5, self.path_planning_cycle)  # 2Hz
        self.control_timer = self.create_timer(0.05, self.navigation_control_cycle)  # 20Hz
        self.recovery_timer = self.create_timer(1.0, self.recovery_check_cycle)  # 1Hz
        
        self.get_logger().info("🧠 ADVANCED NAVIGATION STACK: Multi-sensor fusion navigation started!")
        self.get_logger().info("⚡ Features: LiDAR+Depth fusion, Slope-aware planning, Energy optimization, Auto-recovery")
        
    def lidar_callback(self, msg):
        """Process LiDAR data"""
        self.lidar_data = msg
        
    def depth_callback(self, msg):
        """Process RGB-D point cloud data"""
        self.depth_data = msg
        
    def image_callback(self, msg):
        """Process RGB camera data"""
        try:
            self.rgb_data = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().warn(f"Image conversion error: {e}")
            
    def imu_callback(self, msg):
        """Process IMU data"""
        self.imu_data = msg
        
    def odom_callback(self, msg):
        """Process odometry data"""
        self.current_pose = msg.pose.pose
        
        # Track position history for stuck detection
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_positions.append(current_pos)
        
        if len(self.last_positions) > 20:  # Keep last 20 positions
            self.last_positions.pop(0)
            
    def goal_callback(self, msg):
        """Receive navigation goal"""
        self.goal_pose = msg.pose
        self.get_logger().info(f"🎯 New navigation goal: ({msg.pose.position.x:.1f}, {msg.pose.position.y:.1f})")
        
    def sensor_fusion_cycle(self):
        """🔄 MAIN SENSOR FUSION CYCLE"""
        
        if not all([self.lidar_data, self.current_pose]):
            return
            
        # ═══ FUSE LIDAR AND DEPTH DATA ═══
        obstacles = self.fuse_lidar_depth_obstacles()
        
        # ═══ COMPUTER VISION OBSTACLE DETECTION ═══
        if self.rgb_data is not None:
            vision_obstacles = self.detect_vision_obstacles()
            obstacles.extend(vision_obstacles)
            
        # ═══ UPDATE DYNAMIC COSTMAP ═══
        self.update_dynamic_costmap(obstacles)
        
        # ═══ SLOPE ANALYSIS ═══
        self.analyze_terrain_slopes()
        
        # ═══ PUBLISH RESULTS ═══
        self.publish_fusion_results(obstacles)
        
    def fuse_lidar_depth_obstacles(self):
        """Fuse LiDAR and depth camera for obstacle detection"""
        
        obstacles = []
        
        # ═══ LIDAR OBSTACLE DETECTION ═══
        lidar_obstacles = self.detect_lidar_obstacles()
        obstacles.extend(lidar_obstacles)
        
        # ═══ DEPTH CAMERA OBSTACLE DETECTION ═══
        if self.depth_data:
            depth_obstacles = self.detect_depth_obstacles()
            
            # Fuse with LiDAR (confidence weighting)
            fused_obstacles = self.confidence_weighted_fusion(lidar_obstacles, depth_obstacles)
            obstacles = fused_obstacles
            
        return obstacles
        
    def detect_lidar_obstacles(self):
        """Detect obstacles from LiDAR data"""
        
        obstacles = []
        
        if not self.lidar_data.ranges:
            return obstacles
            
        # Point cloud clustering for obstacle detection
        points = []
        for i, r in enumerate(self.lidar_data.ranges):
            if 0.1 < r < self.lidar_data.range_max:
                angle = self.lidar_data.angle_min + i * self.lidar_data.angle_increment
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
                
        if len(points) > 10:
            # DBSCAN clustering
            points_array = np.array(points)
            clustering = DBSCAN(eps=0.5, min_samples=5).fit(points_array)
            
            # Process clusters as obstacles
            labels = clustering.labels_
            unique_labels = set(labels) - {-1}  # Remove noise label
            
            for label in unique_labels:
                cluster_points = points_array[labels == label]
                
                # Calculate cluster properties
                center_x = np.mean(cluster_points[:, 0])
                center_y = np.mean(cluster_points[:, 1])
                size = np.max(np.linalg.norm(cluster_points - [center_x, center_y], axis=1))
                
                obstacles.append({
                    'type': 'lidar_obstacle',
                    'position': (center_x, center_y),
                    'size': size,
                    'confidence': 0.8,
                    'points': cluster_points.tolist()
                })
                
        return obstacles
        
    def detect_depth_obstacles(self):
        """Detect obstacles from depth camera point cloud"""
        
        obstacles = []
        
        if not self.depth_data:
            return obstacles
            
        try:
            # Convert point cloud to numpy array
            points = list(pc2.read_points(self.depth_data, skip_nans=True, field_names=("x", "y", "z")))
            
            if len(points) < 100:
                return obstacles
                
            points_array = np.array(points)
            
            # Filter points (remove ground plane and distant points)
            valid_points = points_array[
                (points_array[:, 2] > 0.1) &  # Above ground
                (points_array[:, 2] < 5.0) &   # Not too high
                (np.linalg.norm(points_array[:, :2], axis=1) < 20.0)  # Within range
            ]
            
            if len(valid_points) > 20:
                # 3D clustering
                clustering = DBSCAN(eps=0.8, min_samples=10).fit(valid_points)
                labels = clustering.labels_
                unique_labels = set(labels) - {-1}
                
                for label in unique_labels:
                    cluster_points = valid_points[labels == label]
                    
                    # Project to 2D for navigation
                    center_x = np.mean(cluster_points[:, 0])
                    center_y = np.mean(cluster_points[:, 1])
                    center_z = np.mean(cluster_points[:, 2])
                    
                    # Size calculation
                    size = np.max(np.linalg.norm(cluster_points[:, :2] - [center_x, center_y], axis=1))
                    
                    obstacles.append({
                        'type': 'depth_obstacle',
                        'position': (center_x, center_y),
                        'size': size,
                        'height': center_z,
                        'confidence': 0.7,
                        'points_3d': cluster_points.tolist()
                    })
                    
        except Exception as e:
            self.get_logger().warn(f"Depth processing error: {e}")
            
        return obstacles
        
    def detect_vision_obstacles(self):
        """Computer vision obstacle detection"""
        
        obstacles = []
        
        if self.rgb_data is None:
            return obstacles
            
        try:
            # Convert to different color spaces
            gray = cv2.cvtColor(self.rgb_data, cv2.COLOR_BGR2GRAY)
            hsv = cv2.cvtColor(self.rgb_data, cv2.COLOR_BGR2HSV)
            
            # Edge detection for rocks and boulders
            edges = cv2.Canny(gray, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Process significant contours
            height, width = gray.shape
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                if 500 < area < 50000:  # Filter by size
                    # Get contour properties
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Convert image coordinates to world coordinates (simplified)
                    # This would normally use camera calibration
                    center_x_img = x + w // 2
                    center_y_img = y + h // 2
                    
                    # Rough conversion (would use proper camera projection)
                    world_x = (center_x_img - width // 2) * 0.02  # Rough scale
                    world_y = 5.0 - (center_y_img / height) * 10.0  # Distance estimate
                    
                    obstacles.append({
                        'type': 'vision_obstacle',
                        'position': (world_x, world_y),
                        'size': math.sqrt(area) * 0.01,  # Rough size estimate
                        'confidence': 0.6,
                        'contour': contour.tolist()
                    })
                    
        except Exception as e:
            self.get_logger().warn(f"Vision processing error: {e}")
            
        return obstacles
        
    def confidence_weighted_fusion(self, lidar_obstacles, depth_obstacles):
        """Fuse obstacles with confidence weighting"""
        
        fused_obstacles = []
        
        # Start with LiDAR obstacles (higher confidence)
        for lidar_obs in lidar_obstacles:
            fused_obstacles.append(lidar_obs)
            
        # Add depth obstacles that don't overlap with LiDAR
        for depth_obs in depth_obstacles:
            depth_pos = depth_obs['position']
            
            # Check for overlap with existing LiDAR obstacles
            overlaps = False
            for existing_obs in fused_obstacles:
                existing_pos = existing_obs['position']
                distance = math.sqrt(
                    (depth_pos[0] - existing_pos[0])**2 + 
                    (depth_pos[1] - existing_pos[1])**2
                )
                
                if distance < 1.0:  # 1 meter overlap threshold
                    overlaps = True
                    
                    # Update existing obstacle with fused information
                    existing_obs['confidence'] = min(1.0, existing_obs['confidence'] + depth_obs['confidence'] * 0.3)
                    if 'height' not in existing_obs and 'height' in depth_obs:
                        existing_obs['height'] = depth_obs['height']
                    break
                    
            if not overlaps:
                fused_obstacles.append(depth_obs)
                
        return fused_obstacles
        
    def update_dynamic_costmap(self, obstacles):
        """Update dynamic costmap with detected obstacles"""
        
        if not self.current_pose:
            return
            
        # Clear previous dynamic obstacles
        self.local_costmap.fill(0)
        
        # Robot position in costmap coordinates
        robot_x = int(self.current_pose.position.x * 10 + 100)  # Scale and offset
        robot_y = int(self.current_pose.position.y * 10 + 100)
        
        # Add obstacles to costmap
        for obstacle in obstacles:
            obs_x = int(obstacle['position'][0] * 10 + 100)
            obs_y = int(obstacle['position'][1] * 10 + 100)
            obs_size = int(obstacle['size'] * 10) + 5  # Add safety margin
            
            # Ensure within bounds
            if 0 <= obs_x < 200 and 0 <= obs_y < 200:
                # Create circular obstacle
                y_coords, x_coords = np.ogrid[:200, :200]
                mask = (x_coords - obs_x)**2 + (y_coords - obs_y)**2 <= obs_size**2
                
                # Set cost based on confidence
                cost = int(obstacle['confidence'] * 100)
                self.local_costmap[mask] = cost
                
    def analyze_terrain_slopes(self):
        """Analyze terrain slopes for navigation safety"""
        
        if not self.imu_data:
            return
            
        # Get current orientation from IMU
        accel_x = self.imu_data.linear_acceleration.x
        accel_y = self.imu_data.linear_acceleration.y
        accel_z = self.imu_data.linear_acceleration.z
        
        # Calculate slope angles
        slope_x = math.atan2(accel_x, accel_z)
        slope_y = math.atan2(accel_y, accel_z)
        
        total_slope = math.sqrt(slope_x**2 + slope_y**2)
        
        # Update slope information
        if self.current_pose:
            robot_x = int(self.current_pose.position.x * 10 + 500)
            robot_y = int(self.current_pose.position.y * 10 + 500)
            
            if 0 <= robot_x < 1000 and 0 <= robot_y < 1000:
                self.slope_map[robot_y, robot_x] = total_slope
                
        # Publish slope analysis
        slope_status = f"SLOPE: {math.degrees(total_slope):.1f}° | SAFE: {'✅' if total_slope < self.slope_threshold else '❌'}"
        slope_msg = String()
        slope_msg.data = slope_status
        self.slope_pub.publish(slope_msg)
        
    def path_planning_cycle(self):
        """🗺️ PATH PLANNING CYCLE"""
        
        if not self.goal_pose or not self.current_pose:
            return
            
        # ═══ ENERGY-AWARE A* PLANNING ═══
        path = self.energy_aware_astar_planning()
        
        if path:
            self.current_path = path
            self.publish_path(path)
            
        # ═══ CHECK PATH VALIDITY ═══
        if self.current_path and not self.is_path_valid():
            self.get_logger().warn("⚠️ Path blocked, replanning...")
            self.current_path = []
            
    def energy_aware_astar_planning(self):
        """A* path planning with energy optimization"""
        
        if not self.current_pose or not self.goal_pose:
            return None
            
        # Convert positions to grid coordinates
        start_x = int(self.current_pose.position.x * 10 + 100)
        start_y = int(self.current_pose.position.y * 10 + 100)
        goal_x = int(self.goal_pose.position.x * 10 + 100)
        goal_y = int(self.goal_pose.position.y * 10 + 100)
        
        # Boundary checks
        if not (0 <= start_x < 200 and 0 <= start_y < 200 and 
                0 <= goal_x < 200 and 0 <= goal_y < 200):
            return None
            
        # A* algorithm implementation
        open_set = [(0, (start_x, start_y))]
        came_from = {}
        g_score = defaultdict(lambda: float('inf'))
        g_score[(start_x, start_y)] = 0
        
        f_score = defaultdict(lambda: float('inf'))
        f_score[(start_x, start_y)] = self.heuristic_with_energy((start_x, start_y), (goal_x, goal_y))
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            
            if current == (goal_x, goal_y):
                # Reconstruct path
                path = []
                while current in came_from:
                    world_x = (current[0] - 100) / 10.0
                    world_y = (current[1] - 100) / 10.0
                    path.append((world_x, world_y))
                    current = came_from[current]
                path.reverse()
                return path
                
            for neighbor in self.get_neighbors(current):
                if self.is_occupied_with_slopes(neighbor):
                    continue
                    
                # Calculate cost with energy considerations
                movement_cost = self.calculate_movement_cost(current, neighbor)
                tentative_g_score = g_score[current] + movement_cost
                
                if tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.heuristic_with_energy(neighbor, (goal_x, goal_y))
                    
                    if neighbor not in [item[1] for item in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))
                        
        return None  # No path found
        
    def heuristic_with_energy(self, pos, goal):
        """Heuristic function with energy considerations"""
        
        # Euclidean distance
        distance = math.sqrt((goal[0] - pos[0])**2 + (goal[1] - pos[1])**2)
        
        # Add slope penalty
        if 0 <= pos[0] < 1000 and 0 <= pos[1] < 1000:
            slope_penalty = self.slope_map[pos[1], pos[0]] * 50  # Scale slope impact
        else:
            slope_penalty = 0
            
        # Add energy cost
        energy_factor = (100.0 - self.battery_level) / 100.0  # Favor shorter paths when low battery
        
        return distance + slope_penalty + energy_factor * distance
        
    def calculate_movement_cost(self, current, neighbor):
        """Calculate movement cost between two grid cells"""
        
        # Base movement cost
        dx = abs(neighbor[0] - current[0])
        dy = abs(neighbor[1] - current[1])
        base_cost = math.sqrt(dx*dx + dy*dy)
        
        # Obstacle avoidance cost
        if 0 <= neighbor[0] < 200 and 0 <= neighbor[1] < 200:
            obstacle_cost = self.local_costmap[neighbor[1], neighbor[0]] / 100.0
        else:
            obstacle_cost = 0
            
        # Slope cost
        if 0 <= neighbor[0] + 400 < 1000 and 0 <= neighbor[1] + 400 < 1000:
            slope_cost = self.slope_map[neighbor[1] + 400, neighbor[0] + 400] * 10
        else:
            slope_cost = 0
            
        return base_cost + obstacle_cost + slope_cost
        
    def get_neighbors(self, pos):
        """Get neighboring cells for path planning"""
        
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                neighbor = (pos[0] + dx, pos[1] + dy)
                if 0 <= neighbor[0] < 200 and 0 <= neighbor[1] < 200:
                    neighbors.append(neighbor)
        return neighbors
        
    def is_occupied_with_slopes(self, pos):
        """Check if position is occupied or has unsafe slope"""
        
        # Check obstacle occupancy
        if 0 <= pos[0] < 200 and 0 <= pos[1] < 200:
            if self.local_costmap[pos[1], pos[0]] > 80:  # High cost threshold
                return True
                
        # Check slope safety
        slope_x = pos[0] + 400
        slope_y = pos[1] + 400
        if 0 <= slope_x < 1000 and 0 <= slope_y < 1000:
            if self.slope_map[slope_y, slope_x] > self.slope_threshold:
                return True
                
        return False
        
    def is_path_valid(self):
        """Check if current path is still valid"""
        
        if not self.current_path:
            return False
            
        # Check if path intersects with new obstacles
        for waypoint in self.current_path[:5]:  # Check next 5 waypoints
            wp_x = int(waypoint[0] * 10 + 100)
            wp_y = int(waypoint[1] * 10 + 100)
            
            if 0 <= wp_x < 200 and 0 <= wp_y < 200:
                if self.local_costmap[wp_y, wp_x] > 90:  # Very high cost
                    return False
                    
        return True
        
    def navigation_control_cycle(self):
        """🎮 NAVIGATION CONTROL CYCLE"""
        
        if not self.current_path or not self.current_pose:
            return
            
        if self.recovery_mode:
            self.execute_recovery_behavior()
            return
            
        # ═══ PURE PURSUIT CONTROLLER ═══
        cmd = self.pure_pursuit_control()
        
        # ═══ OBSTACLE AVOIDANCE OVERRIDE ═══
        cmd = self.dynamic_obstacle_avoidance(cmd)
        
        # ═══ ENERGY-AWARE SPEED CONTROL ═══
        cmd = self.energy_aware_speed_control(cmd)
        
        # ═══ PUBLISH COMMAND ═══
        self.cmd_pub.publish(cmd)
        
    def pure_pursuit_control(self):
        """Pure pursuit path following controller"""
        
        cmd = Twist()
        
        if not self.current_path:
            return cmd
            
        # Find lookahead point
        lookahead_distance = 2.0  # 2 meters
        current_pos = (self.current_pose.position.x, self.current_pose.position.y)
        
        target_point = None
        for waypoint in self.current_path:
            distance = math.sqrt(
                (waypoint[0] - current_pos[0])**2 + 
                (waypoint[1] - current_pos[1])**2
            )
            if distance >= lookahead_distance:
                target_point = waypoint
                break
                
        if not target_point and self.current_path:
            target_point = self.current_path[-1]  # Use final goal
            
        if target_point:
            # Calculate steering
            dx = target_point[0] - current_pos[0]
            dy = target_point[1] - current_pos[1]
            
            target_heading = math.atan2(dy, dx)
            
            # Get current heading from pose (simplified)
            current_heading = self.get_yaw_from_pose(self.current_pose)
            
            heading_error = target_heading - current_heading
            
            # Normalize angle
            while heading_error > math.pi:
                heading_error -= 2 * math.pi
            while heading_error < -math.pi:
                heading_error += 2 * math.pi
                
            # Control commands
            distance_to_goal = math.sqrt(dx*dx + dy*dy)
            
            if distance_to_goal < 0.5:  # Close to goal
                cmd.linear.x = 0.1
                cmd.angular.z = 0.0
            else:
                cmd.linear.x = min(0.5, distance_to_goal * 0.3)  # Proportional speed
                cmd.angular.z = heading_error * 0.8  # Proportional steering
                
        return cmd
        
    def dynamic_obstacle_avoidance(self, cmd):
        """Dynamic obstacle avoidance using DWB-like approach"""
        
        if not self.lidar_data:
            return cmd
            
        # Check for immediate obstacles
        front_ranges = []
        num_ranges = len(self.lidar_data.ranges)
        
        # Front-facing LiDAR readings (±45 degrees)
        front_start = int(num_ranges * 3/8)  # -45 degrees
        front_end = int(num_ranges * 5/8)    # +45 degrees
        
        for i in range(front_start, front_end):
            if i < len(self.lidar_data.ranges):
                r = self.lidar_data.ranges[i]
                if 0.1 < r < 10.0:  # Valid range
                    front_ranges.append(r)
                    
        if front_ranges:
            min_distance = min(front_ranges)
            
            if min_distance < 2.0:  # Obstacle within 2 meters
                # Reduce speed
                cmd.linear.x *= (min_distance / 2.0)
                
                if min_distance < 1.0:  # Very close obstacle
                    # Stop and turn
                    cmd.linear.x = 0.0
                    
                    # Choose turn direction based on clearer side
                    left_ranges = self.lidar_data.ranges[front_end:front_end+50]
                    right_ranges = self.lidar_data.ranges[front_start-50:front_start]
                    
                    left_clear = sum(r for r in left_ranges if r > 2.0)
                    right_clear = sum(r for r in right_ranges if r > 2.0)
                    
                    if left_clear > right_clear:
                        cmd.angular.z = 0.5  # Turn left
                    else:
                        cmd.angular.z = -0.5  # Turn right
                        
        return cmd
        
    def energy_aware_speed_control(self, cmd):
        """Adjust speed based on battery level"""
        
        if self.battery_level < 30:  # Low battery
            cmd.linear.x *= 0.6  # Reduce speed by 40%
            cmd.angular.z *= 0.8  # Reduce turning speed
        elif self.battery_level < 50:  # Moderate battery
            cmd.linear.x *= 0.8  # Reduce speed by 20%
            
        return cmd
        
    def recovery_check_cycle(self):
        """🔄 AUTO-RECOVERY CHECK CYCLE"""
        
        # Check if robot is stuck
        if len(self.last_positions) >= 10:
            recent_positions = self.last_positions[-10:]
            
            # Calculate movement in last 10 seconds
            start_pos = recent_positions[0]
            end_pos = recent_positions[-1]
            
            distance_moved = math.sqrt(
                (end_pos[0] - start_pos[0])**2 + 
                (end_pos[1] - start_pos[1])**2
            )
            
            if distance_moved < 0.5:  # Less than 0.5m in 10 seconds
                self.stuck_counter += 1
            else:
                self.stuck_counter = 0
                
            # Trigger recovery if stuck for too long
            if self.stuck_counter > 5 and not self.recovery_mode:  # 5 seconds
                self.recovery_mode = True
                self.get_logger().warn("🔄 AUTO-RECOVERY: Robot appears stuck, initiating recovery!")
                
        # Publish recovery status
        recovery_msg = Bool()
        recovery_msg.data = self.recovery_mode
        self.recovery_pub.publish(recovery_msg)
        
    def execute_recovery_behavior(self):
        """Execute autonomous recovery behavior"""
        
        cmd = Twist()
        
        # Recovery sequence: back up, turn, try forward
        recovery_phase = self.stuck_counter % 12
        
        if recovery_phase < 4:  # Phase 1: Back up
            cmd.linear.x = -0.2
            cmd.angular.z = 0.0
            self.get_logger().info("🔄 RECOVERY Phase 1: Backing up")
            
        elif recovery_phase < 8:  # Phase 2: Turn
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5
            self.get_logger().info("🔄 RECOVERY Phase 2: Turning")
            
        else:  # Phase 3: Try forward
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0
            self.get_logger().info("🔄 RECOVERY Phase 3: Moving forward")
            
            # Check if recovery successful
            if recovery_phase >= 11:
                self.recovery_mode = False
                self.stuck_counter = 0
                self.current_path = []  # Clear path to replan
                self.get_logger().info("✅ RECOVERY: Completed, resuming normal navigation")
                
        self.cmd_pub.publish(cmd)
        
    def get_yaw_from_pose(self, pose):
        """Extract yaw angle from pose"""
        
        # Convert quaternion to yaw (simplified)
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def publish_path(self, path):
        """Publish planned path"""
        
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for waypoint in path:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = "map"
            pose_stamped.pose.position.x = waypoint[0]
            pose_stamped.pose.position.y = waypoint[1]
            pose_stamped.pose.position.z = 0.0
            path_msg.poses.append(pose_stamped)
            
        self.path_pub.publish(path_msg)
        
    def publish_fusion_results(self, obstacles):
        """Publish sensor fusion results"""
        
        # ═══ PUBLISH COSTMAP ═══
        costmap_msg = OccupancyGrid()
        costmap_msg.header.frame_id = "base_link"
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.info.resolution = 0.1
        costmap_msg.info.width = 200
        costmap_msg.info.height = 200
        costmap_msg.info.origin.position.x = -10.0
        costmap_msg.info.origin.position.y = -10.0
        costmap_msg.data = self.local_costmap.flatten().astype(np.int8).tolist()
        self.costmap_pub.publish(costmap_msg)
        
        # ═══ PUBLISH OBSTACLE SUMMARY ═══
        obstacle_summary = f"OBSTACLES: Total:{len(obstacles)} | LiDAR:{len([o for o in obstacles if o['type']=='lidar_obstacle'])} | Depth:{len([o for o in obstacles if o['type']=='depth_obstacle'])} | Vision:{len([o for o in obstacles if o['type']=='vision_obstacle'])}"
        
        obstacles_msg = String()
        obstacles_msg.data = obstacle_summary
        self.obstacles_pub.publish(obstacles_msg)
        
        # ═══ PUBLISH NAVIGATION STATUS ═══
        nav_status = f"PATH: {len(self.current_path)} waypoints | GOAL: {'✅' if self.goal_pose else '❌'} | RECOVERY: {'🔄' if self.recovery_mode else '✅'}"
        
        nav_msg = String()
        nav_msg.data = nav_status
        self.nav_status_pub.publish(nav_msg)
        
        # ═══ PUBLISH ENERGY STATUS ═══
        energy_status = f"BATTERY: {self.battery_level:.1f}% | ENERGY_PLANNING: {'✅' if self.battery_level > 30 else '⚠️'}"
        
        energy_msg = String()
        energy_msg.data = energy_status
        self.energy_pub.publish(energy_msg)

def main():
    rclpy.init()
    nav_stack = AdvancedNavigationStack()
    
    try:
        rclpy.spin(nav_stack)
    except KeyboardInterrupt:
        nav_stack.get_logger().info("🧠 Advanced Navigation Stack stopped")
    finally:
        nav_stack.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
