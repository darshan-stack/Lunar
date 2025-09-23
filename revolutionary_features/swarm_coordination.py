#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Float32, Bool, Int32MultiArray
import math
import time
import random
import numpy as np
import json

class SwarmCoordinationSystem(Node):
    """🦋 Revolutionary Multi-Robot Coordination & Swarm Behavior"""
    
    def __init__(self):
        super().__init__('swarm_coordination_system')
        
        # ═══ SWARM CONFIGURATION ═══
        self.robot_id = self.declare_parameter('robot_id', 0).get_parameter_value().integer_value
        self.swarm_size = 3  # 3 robots in swarm
        self.swarm_robots = {}  # Track other robots
        
        # ═══ FIREFLY-INSPIRED SYNCHRONIZATION ═══
        self.firefly_phase = random.uniform(0, 2 * math.pi)  # Random initial phase
        self.firefly_frequency = 1.0  # Base frequency
        self.sync_threshold = 0.1  # Synchronization sensitivity
        self.flash_intensity = 0.0  # Current flash brightness
        self.last_flash_time = 0
        
        # ═══ SWARM STATE ═══
        self.swarm_mode = 'exploration'  # exploration, coordination, emergency, formation
        self.formation_type = 'triangle'  # triangle, line, circle
        self.formation_spacing = 10.0  # meters between robots
        self.leader_id = 0  # Current swarm leader
        
        # ═══ COORDINATION METRICS ═══
        self.coordination_metrics = {
            'sync_level': 0.0,
            'formation_error': 0.0,
            'communication_quality': 1.0,
            'collective_efficiency': 0.0,
            'task_distribution': 0.0,
            'swarm_health': 100.0
        }
        
        # ═══ TASK ALLOCATION ═══
        self.assigned_tasks = []
        self.completed_tasks = []
        self.shared_map = None
        self.exploration_targets = []
        
        # ═══ ROBOT STATE ═══
        self.current_pose = None
        self.current_battery = 100.0
        self.robot_health = 100.0
        self.is_leader = (self.robot_id == self.leader_id)
        
        # ═══ COMMUNICATION SIMULATION ═══
        self.communication_range = 50.0  # meters
        self.message_queue = []
        self.last_broadcast_time = 0
        
        # ═══ ROS SUBSCRIBERS ═══
        self.pose_sub = self.create_subscription(PoseStamped, '/adaptive_slam_pose', self.pose_callback, 10)
        self.battery_sub = self.create_subscription(Float32, '/battery_level', self.battery_callback, 10)
        self.health_sub = self.create_subscription(String, '/system_health', self.health_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/adaptive_slam_map', self.map_callback, 10)
        
        # Inter-robot communication (simulated via topics)
        for i in range(self.swarm_size):
            if i != self.robot_id:
                topic_name = f'/robot_{i}/swarm_data'
                self.create_subscription(String, topic_name, lambda msg, rid=i: self.swarm_data_callback(msg, rid), 10)
                
        # ═══ ROS PUBLISHERS ═══
        self.swarm_status_pub = self.create_publisher(String, '/swarm_status', 10)
        self.formation_pub = self.create_publisher(String, '/formation_status', 10)
        self.firefly_pub = self.create_publisher(Float32, '/firefly_sync', 10)
        self.coordination_pub = self.create_publisher(String, '/coordination_metrics', 10)
        self.task_pub = self.create_publisher(String, '/swarm_tasks', 10)
        self.swarm_cmd_pub = self.create_publisher(Twist, '/swarm_cmd_vel', 10)
        
        # Broadcast own data to other robots
        self.broadcast_pub = self.create_publisher(String, f'/robot_{self.robot_id}/swarm_data', 10)
        
        # ═══ TIMERS ═══
        self.firefly_timer = self.create_timer(0.1, self.firefly_synchronization_cycle)  # 10Hz firefly sync
        self.coordination_timer = self.create_timer(0.5, self.coordination_cycle)  # 2Hz coordination
        self.task_allocation_timer = self.create_timer(2.0, self.task_allocation_cycle)  # 0.5Hz task allocation
        self.formation_timer = self.create_timer(1.0, self.formation_control_cycle)  # 1Hz formation
        
        self.start_time = time.time()
        
        # ═══ INITIALIZE SWARM ═══
        self.initialize_swarm_robots()
        
        self.get_logger().info(f"🦋 SWARM COORDINATION: Robot {self.robot_id} joining swarm!")
        self.get_logger().info("⚡ Features: Firefly sync, Formation control, Task allocation, Emergency coordination")
        
    def initialize_swarm_robots(self):
        """Initialize virtual swarm robots for simulation"""
        
        for i in range(self.swarm_size):
            if i != self.robot_id:
                # Simulate other robots with random positions
                angle = (2 * math.pi * i) / self.swarm_size
                self.swarm_robots[i] = {
                    'id': i,
                    'pose': {
                        'x': 15 * math.cos(angle),
                        'y': 15 * math.sin(angle),
                        'theta': angle
                    },
                    'battery': random.uniform(70, 100),
                    'health': random.uniform(85, 100),
                    'firefly_phase': random.uniform(0, 2 * math.pi),
                    'last_update': time.time(),
                    'communication_quality': 1.0,
                    'assigned_tasks': [],
                    'status': 'active'
                }
                
        self.get_logger().info(f"🤖 Swarm initialized: {self.swarm_size} robots")
        
    def pose_callback(self, msg):
        """Update current robot pose"""
        
        self.current_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'theta': self.quaternion_to_yaw(msg.pose.orientation)
        }
        
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        
        # Simplified conversion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def battery_callback(self, msg):
        """Update current battery level"""
        
        self.current_battery = msg.data
        
    def health_callback(self, msg):
        """Update robot health status"""
        
        # Extract health from status message
        if 'HEALTH:' in msg.data:
            try:
                health_str = msg.data.split('HEALTH:')[1].split('%')[0]
                self.robot_health = float(health_str)
            except:
                pass
                
    def map_callback(self, msg):
        """Update shared map data"""
        
        # In real implementation, would merge maps from different robots
        self.shared_map = msg
        
    def swarm_data_callback(self, msg, robot_id):
        """Receive data from other robots"""
        
        try:
            data = json.loads(msg.data)
            
            # Update robot information
            if robot_id in self.swarm_robots:
                self.swarm_robots[robot_id].update(data)
                self.swarm_robots[robot_id]['last_update'] = time.time()
                
        except json.JSONDecodeError:
            pass
            
    def firefly_synchronization_cycle(self):
        """🌟 FIREFLY-INSPIRED SYNCHRONIZATION"""
        
        current_time = time.time()
        dt = 0.1  # Timer interval
        
        # ═══ UPDATE FIREFLY PHASE ═══
        self.firefly_phase += 2 * math.pi * self.firefly_frequency * dt
        self.firefly_phase = self.firefly_phase % (2 * math.pi)
        
        # ═══ CALCULATE COUPLING INFLUENCE ═══
        coupling_strength = 0.1
        phase_adjustment = 0.0
        
        for robot_id, robot in self.swarm_robots.items():
            if time.time() - robot['last_update'] < 5.0:  # Recent communication
                # Calculate phase difference
                phase_diff = robot['firefly_phase'] - self.firefly_phase
                
                # Normalize phase difference to [-π, π]
                while phase_diff > math.pi:
                    phase_diff -= 2 * math.pi
                while phase_diff < -math.pi:
                    phase_diff += 2 * math.pi
                    
                # Kuramoto coupling
                phase_adjustment += coupling_strength * math.sin(phase_diff)
                
        # ═══ APPLY COUPLING ═══
        self.firefly_phase += phase_adjustment * dt
        self.firefly_phase = self.firefly_phase % (2 * math.pi)
        
        # ═══ CALCULATE FLASH INTENSITY ═══
        self.flash_intensity = max(0, math.sin(self.firefly_phase))
        
        # ═══ DETECT FLASH EVENT ═══
        if (self.flash_intensity > 0.9 and 
            current_time - self.last_flash_time > 0.5):
            
            self.last_flash_time = current_time
            self.get_logger().info("✨ FIREFLY FLASH: Synchronization pulse")
            
        # ═══ PUBLISH SYNC STATUS ═══
        sync_msg = Float32()
        sync_msg.data = self.flash_intensity
        self.firefly_pub.publish(sync_msg)
        
    def coordination_cycle(self):
        """🤝 MAIN COORDINATION CYCLE"""
        
        # ═══ BROADCAST ROBOT DATA ═══
        self.broadcast_robot_data()
        
        # ═══ UPDATE COORDINATION METRICS ═══
        self.update_coordination_metrics()
        
        # ═══ DETERMINE SWARM MODE ═══
        self.determine_swarm_mode()
        
        # ═══ EXECUTE COORDINATION STRATEGY ═══
        if self.swarm_mode == 'exploration':
            self.execute_exploration_coordination()
        elif self.swarm_mode == 'formation':
            self.execute_formation_coordination()
        elif self.swarm_mode == 'emergency':
            self.execute_emergency_coordination()
        elif self.swarm_mode == 'coordination':
            self.execute_collaborative_coordination()
            
        # ═══ PUBLISH STATUS ═══
        self.publish_swarm_status()
        
    def broadcast_robot_data(self):
        """📡 BROADCAST ROBOT DATA TO SWARM"""
        
        if not self.current_pose:
            return
            
        robot_data = {
            'id': self.robot_id,
            'pose': self.current_pose,
            'battery': self.current_battery,
            'health': self.robot_health,
            'firefly_phase': self.firefly_phase,
            'assigned_tasks': self.assigned_tasks,
            'status': 'active' if self.robot_health > 50 else 'degraded',
            'timestamp': time.time()
        }
        
        broadcast_msg = String()
        broadcast_msg.data = json.dumps(robot_data)
        self.broadcast_pub.publish(broadcast_msg)
        
    def update_coordination_metrics(self):
        """📊 UPDATE COORDINATION PERFORMANCE METRICS"""
        
        active_robots = [r for r in self.swarm_robots.values() 
                        if time.time() - r['last_update'] < 10.0]
        
        if len(active_robots) == 0:
            return
            
        # ═══ SYNCHRONIZATION LEVEL ═══
        if len(active_robots) > 0:
            phase_differences = []
            for robot in active_robots:
                phase_diff = abs(robot['firefly_phase'] - self.firefly_phase)
                phase_diff = min(phase_diff, 2 * math.pi - phase_diff)  # Circular distance
                phase_differences.append(phase_diff)
                
            avg_phase_diff = np.mean(phase_differences)
            self.coordination_metrics['sync_level'] = max(0, 1.0 - avg_phase_diff / math.pi)
            
        # ═══ FORMATION ERROR ═══
        self.coordination_metrics['formation_error'] = self.calculate_formation_error()
        
        # ═══ COMMUNICATION QUALITY ═══
        comm_scores = [r['communication_quality'] for r in active_robots]
        self.coordination_metrics['communication_quality'] = np.mean(comm_scores) if comm_scores else 1.0
        
        # ═══ SWARM HEALTH ═══
        health_scores = [r['health'] for r in active_robots] + [self.robot_health]
        self.coordination_metrics['swarm_health'] = np.mean(health_scores)
        
        # ═══ COLLECTIVE EFFICIENCY ═══
        completed_ratio = len(self.completed_tasks) / max(len(self.assigned_tasks), 1)
        self.coordination_metrics['collective_efficiency'] = completed_ratio * self.coordination_metrics['sync_level']
        
    def calculate_formation_error(self):
        """Calculate formation maintenance error"""
        
        if not self.current_pose or len(self.swarm_robots) < 2:
            return 0.0
            
        if self.formation_type == 'triangle':
            return self.calculate_triangle_formation_error()
        elif self.formation_type == 'line':
            return self.calculate_line_formation_error()
        else:
            return 0.0
            
    def calculate_triangle_formation_error(self):
        """Calculate error for triangle formation"""
        
        if len(self.swarm_robots) < 2:
            return 0.0
            
        # Get positions of active robots
        positions = [self.current_pose]
        for robot in self.swarm_robots.values():
            if time.time() - robot['last_update'] < 5.0:
                positions.append(robot['pose'])
                
        if len(positions) < 3:
            return 0.5  # Incomplete formation
            
        # Calculate distances between robots
        distances = []
        for i in range(len(positions)):
            for j in range(i + 1, len(positions)):
                p1, p2 = positions[i], positions[j]
                dist = math.sqrt((p1['x'] - p2['x'])**2 + (p1['y'] - p2['y'])**2)
                distances.append(dist)
                
        # Error is deviation from desired spacing
        target_distance = self.formation_spacing
        distance_errors = [abs(d - target_distance) / target_distance for d in distances]
        
        return np.mean(distance_errors) if distance_errors else 0.0
        
    def calculate_line_formation_error(self):
        """Calculate error for line formation"""
        
        # Simplified line formation error calculation
        return random.uniform(0, 0.3)  # Placeholder
        
    def determine_swarm_mode(self):
        """🧠 DETERMINE OPTIMAL SWARM MODE"""
        
        # ═══ EMERGENCY CONDITIONS ═══
        if (self.robot_health < 50 or 
            self.current_battery < 20 or
            any(r['health'] < 30 for r in self.swarm_robots.values())):
            
            self.swarm_mode = 'emergency'
            
        # ═══ FORMATION CONDITIONS ═══
        elif (self.coordination_metrics['formation_error'] > 0.5 or
              self.coordination_metrics['sync_level'] < 0.3):
            
            self.swarm_mode = 'formation'
            
        # ═══ COORDINATION CONDITIONS ═══
        elif len(self.assigned_tasks) > 0:
            self.swarm_mode = 'coordination'
            
        # ═══ DEFAULT EXPLORATION ═══
        else:
            self.swarm_mode = 'exploration'
            
    def execute_exploration_coordination(self):
        """🔍 COORDINATED EXPLORATION STRATEGY"""
        
        # Assign exploration sectors to different robots
        if self.is_leader:
            self.assign_exploration_sectors()
            
        # Generate exploration movement
        if self.current_pose:
            exploration_cmd = self.generate_exploration_command()
            self.swarm_cmd_pub.publish(exploration_cmd)
            
    def execute_formation_coordination(self):
        """🔺 FORMATION CONTROL COORDINATION"""
        
        if not self.current_pose:
            return
            
        # Calculate desired position in formation
        desired_pos = self.calculate_desired_formation_position()
        
        if desired_pos:
            # Generate movement command toward desired position
            formation_cmd = self.generate_formation_command(desired_pos)
            self.swarm_cmd_pub.publish(formation_cmd)
            
    def execute_emergency_coordination(self):
        """🚨 EMERGENCY COORDINATION PROTOCOL"""
        
        self.get_logger().warn("🚨 SWARM EMERGENCY: Coordinated emergency response")
        
        # Emergency behaviors:
        # 1. Help struggling robots
        # 2. Redistribute tasks
        # 3. Form protective formation
        
        if self.robot_health > 70:  # Healthy robot helps others
            struggling_robot = self.find_struggling_robot()
            if struggling_robot:
                self.get_logger().info(f"🆘 HELPING: Robot {struggling_robot['id']}")
                
        # Generate conservative movement command
        emergency_cmd = Twist()
        emergency_cmd.linear.x = 0.1  # Slow movement
        emergency_cmd.angular.z = 0.0
        self.swarm_cmd_pub.publish(emergency_cmd)
        
    def execute_collaborative_coordination(self):
        """🤝 COLLABORATIVE TASK COORDINATION"""
        
        # Execute assigned tasks collaboratively
        if self.assigned_tasks:
            current_task = self.assigned_tasks[0]
            self.execute_collaborative_task(current_task)
            
    def assign_exploration_sectors(self):
        """📍 ASSIGN EXPLORATION SECTORS TO ROBOTS"""
        
        if not self.is_leader:
            return
            
        # Simple sector assignment based on robot positions
        sector_angles = [
            0,              # North
            2 * math.pi / 3,    # 120 degrees
            4 * math.pi / 3     # 240 degrees
        ]
        
        active_robots = list(self.swarm_robots.keys()) + [self.robot_id]
        
        for i, robot_id in enumerate(active_robots[:3]):
            if robot_id == self.robot_id:
                self.exploration_target_angle = sector_angles[i]
            else:
                # In real implementation, would send task via communication
                pass
                
    def find_struggling_robot(self):
        """🔍 FIND ROBOT THAT NEEDS HELP"""
        
        for robot in self.swarm_robots.values():
            if (robot['health'] < 40 or 
                robot['battery'] < 15 or
                robot['status'] == 'degraded'):
                return robot
                
        return None
        
    def calculate_desired_formation_position(self):
        """📐 CALCULATE DESIRED POSITION IN FORMATION"""
        
        if self.formation_type == 'triangle':
            # Triangle formation with robot_id determining position
            angles = [0, 2 * math.pi / 3, 4 * math.pi / 3]
            angle = angles[self.robot_id % 3]
            
            # Formation center (leader position or centroid)
            if self.leader_id in self.swarm_robots:
                center = self.swarm_robots[self.leader_id]['pose']
            else:
                center = self.current_pose
                
            if center:
                desired_x = center['x'] + self.formation_spacing * math.cos(angle)
                desired_y = center['y'] + self.formation_spacing * math.sin(angle)
                
                return {'x': desired_x, 'y': desired_y}
                
        return None
        
    def generate_exploration_command(self):
        """🎯 GENERATE EXPLORATION MOVEMENT COMMAND"""
        
        cmd = Twist()
        
        # Simple exploration pattern
        current_time = time.time() - self.start_time
        
        # Use firefly sync to coordinate movement patterns
        sync_factor = math.sin(self.firefly_phase)
        
        cmd.linear.x = 0.2 + 0.1 * sync_factor
        cmd.angular.z = 0.1 * math.sin(current_time * 0.1 + self.robot_id)
        
        return cmd
        
    def generate_formation_command(self, desired_pos):
        """🔺 GENERATE FORMATION CONTROL COMMAND"""
        
        if not self.current_pose:
            return Twist()
            
        cmd = Twist()
        
        # Proportional controller toward desired position
        dx = desired_pos['x'] - self.current_pose['x']
        dy = desired_pos['y'] - self.current_pose['y']
        
        distance = math.sqrt(dx**2 + dy**2)
        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - self.current_pose['theta']
        
        # Normalize angle error
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        # Control gains
        linear_gain = 0.3
        angular_gain = 0.8
        
        cmd.linear.x = min(0.5, linear_gain * distance)
        cmd.angular.z = angular_gain * angle_error
        
        return cmd
        
    def execute_collaborative_task(self, task):
        """🤝 EXECUTE COLLABORATIVE TASK"""
        
        # Placeholder for task execution
        # In real implementation, would coordinate specific tasks
        self.get_logger().info(f"🎯 EXECUTING TASK: {task}")
        
    def task_allocation_cycle(self):
        """📋 TASK ALLOCATION AND MANAGEMENT"""
        
        current_time = time.time() - self.start_time
        
        # ═══ GENERATE COLLABORATIVE TASKS ═══
        if len(self.assigned_tasks) == 0 and current_time > 10:  # After 10 seconds
            if self.is_leader:
                self.generate_swarm_tasks()
            else:
                # Request tasks from leader
                pass
                
        # ═══ TASK COMPLETION CHECK ═══
        self.check_task_completion()
        
        # ═══ PUBLISH TASK STATUS ═══
        task_status = f"TASKS: Assigned:{len(self.assigned_tasks)} Completed:{len(self.completed_tasks)}"
        task_msg = String()
        task_msg.data = task_status
        self.task_pub.publish(task_msg)
        
    def generate_swarm_tasks(self):
        """📝 GENERATE TASKS FOR SWARM"""
        
        collaborative_tasks = [
            "explore_sector_north",
            "map_obstacle_field", 
            "establish_communication_relay",
            "coordinate_resource_collection"
        ]
        
        # Distribute tasks among robots
        active_robots = [self.robot_id] + list(self.swarm_robots.keys())
        
        for i, task in enumerate(collaborative_tasks):
            robot_id = active_robots[i % len(active_robots)]
            
            if robot_id == self.robot_id:
                self.assigned_tasks.append(task)
            else:
                # In real implementation, would assign via communication
                pass
                
    def check_task_completion(self):
        """✅ CHECK TASK COMPLETION STATUS"""
        
        # Simple task completion simulation
        if self.assigned_tasks and random.random() < 0.1:  # 10% chance per cycle
            completed_task = self.assigned_tasks.pop(0)
            self.completed_tasks.append(completed_task)
            self.get_logger().info(f"✅ TASK COMPLETED: {completed_task}")
            
    def formation_control_cycle(self):
        """🔺 FORMATION CONTROL CYCLE"""
        
        # ═══ UPDATE FORMATION TYPE BASED ON CONDITIONS ═══
        if self.coordination_metrics['sync_level'] > 0.8:
            self.formation_type = 'triangle'  # Tight formation
        else:
            self.formation_type = 'line'      # Loose formation
            
        # ═══ PUBLISH FORMATION STATUS ═══
        formation_status = (
            f"FORMATION:{self.formation_type.upper()} | "
            f"SPACING:{self.formation_spacing:.1f}m | "
            f"ERROR:{self.coordination_metrics['formation_error']:.2f} | "
            f"LEADER:Robot{self.leader_id}"
        )
        
        formation_msg = String()
        formation_msg.data = formation_status
        self.formation_pub.publish(formation_msg)
        
    def publish_swarm_status(self):
        """📊 PUBLISH COMPREHENSIVE SWARM STATUS"""
        
        # ═══ SWARM STATUS MESSAGE ═══
        active_count = len([r for r in self.swarm_robots.values() 
                           if time.time() - r['last_update'] < 10.0]) + 1
        
        status_parts = [
            f"MODE:{self.swarm_mode.upper()}",
            f"ROBOTS:{active_count}/{self.swarm_size}",
            f"SYNC:{self.coordination_metrics['sync_level']:.2f}",
            f"HEALTH:{self.coordination_metrics['swarm_health']:.0f}%"
        ]
        
        if self.is_leader:
            status_parts.append("👑LEADER")
            
        swarm_status_msg = String()
        swarm_status_msg.data = " | ".join(status_parts)
        self.swarm_status_pub.publish(swarm_status_msg)
        
        # ═══ COORDINATION METRICS MESSAGE ═══
        metrics_parts = [
            f"SYNC:{self.coordination_metrics['sync_level']:.2f}",
            f"FORM_ERR:{self.coordination_metrics['formation_error']:.2f}",
            f"COMM:{self.coordination_metrics['communication_quality']:.2f}",
            f"EFF:{self.coordination_metrics['collective_efficiency']:.2f}"
        ]
        
        coordination_msg = String()
        coordination_msg.data = " | ".join(metrics_parts)
        self.coordination_pub.publish(coordination_msg)

def main():
    rclpy.init()
    swarm_system = SwarmCoordinationSystem()
    
    try:
        rclpy.spin(swarm_system)
    except KeyboardInterrupt:
        swarm_system.get_logger().info("🦋 Swarm Coordination System stopped")
    finally:
        swarm_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
