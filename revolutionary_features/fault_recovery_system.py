#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, JointState
from std_msgs.msg import String, Bool, Float32
import time
import math
import random
import numpy as np

class FaultRecoverySystem(Node):
    """🔧 Revolutionary Fault Injection & Autonomous Recovery System"""
    
    def __init__(self):
        super().__init__('fault_recovery_system')
        
        # ═══ SYSTEM HEALTH STATUS ═══
        self.health_status = {
            'left_wheel': {'status': True, 'performance': 1.0},
            'right_wheel': {'status': True, 'performance': 1.0},
            'lidar': {'status': True, 'performance': 1.0},
            'camera': {'status': True, 'performance': 1.0},
            'imu': {'status': True, 'performance': 1.0},
            'battery': {'status': True, 'performance': 1.0},
            'communication': {'status': True, 'performance': 1.0}
        }
        
        # ═══ FAULT INJECTION SCHEDULE ═══
        self.fault_schedule = [
            {'time': 30, 'component': 'left_wheel', 'severity': 0.3, 'triggered': False},
            {'time': 60, 'component': 'lidar', 'severity': 0.7, 'triggered': False},
            {'time': 90, 'component': 'right_wheel', 'severity': 0.5, 'triggered': False},
            {'time': 120, 'component': 'camera', 'severity': 0.8, 'triggered': False},
        ]
        
        # ═══ RECOVERY STATE ═══
        self.in_recovery_mode = False
        self.recovery_start_time = 0
        self.recovery_phase = 0
        self.recovery_target_component = None
        self.recovery_strategy = None
        
        # ═══ MOVEMENT MONITORING ═══
        self.last_positions = []
        self.position_history_size = 30
        self.stuck_threshold = 0.1  # 10cm movement in 6 seconds = stuck
        self.is_stuck = False
        
        # ═══ PERFORMANCE MONITORING ═══
        self.performance_metrics = {
            'total_faults': 0,
            'successful_recoveries': 0,
            'failed_recoveries': 0,
            'recovery_time_avg': 0.0,
            'system_uptime': 0.0
        }
        
        # ═══ ROS SUBSCRIBERS ═══
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # ═══ ROS PUBLISHERS ═══
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fault_status_pub = self.create_publisher(String, '/fault_status', 10)
        self.recovery_pub = self.create_publisher(Bool, '/recovery_mode_active', 10)
        self.health_pub = self.create_publisher(String, '/system_health', 10)
        self.metrics_pub = self.create_publisher(String, '/recovery_metrics', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # ═══ TIMERS ═══
        self.fault_timer = self.create_timer(1.0, self.fault_injection_cycle)
        self.recovery_timer = self.create_timer(0.5, self.recovery_execution_cycle)
        self.health_timer = self.create_timer(2.0, self.health_monitoring_cycle)
        
        self.start_time = time.time()
        
        self.get_logger().info("🔧 FAULT RECOVERY SYSTEM: Revolutionary autonomous recovery started!")
        self.get_logger().info("⚡ Features: Fault injection, Multi-phase recovery, Performance monitoring")
        
    def odom_callback(self, msg):
        """Track position for stuck detection"""
        
        current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.last_positions.append(current_pos)
        
        # Keep only recent positions
        if len(self.last_positions) > self.position_history_size:
            self.last_positions.pop(0)
            
        # Check for stuck condition
        self.check_stuck_condition()
        
    def scan_callback(self, msg):
        """Monitor LiDAR health and obstacles"""
        
        # ═══ LIDAR HEALTH ASSESSMENT ═══
        valid_readings = sum(1 for r in msg.ranges if 0.1 < r < msg.range_max)
        total_readings = len(msg.ranges)
        
        if total_readings > 0:
            health_ratio = valid_readings / total_readings
            self.health_status['lidar']['performance'] = health_ratio
            
            # Update LiDAR health status
            if health_ratio < 0.3:
                self.health_status['lidar']['status'] = False
            elif health_ratio > 0.7:
                self.health_status['lidar']['status'] = True
                
        # ═══ OBSTACLE DETECTION FOR RECOVERY ═══
        if self.in_recovery_mode and self.recovery_strategy == 'obstacle_avoidance':
            self.execute_obstacle_recovery(msg)
            
    def joint_callback(self, msg):
        """Monitor wheel joint health"""
        
        # Simulate wheel health monitoring
        # In real system, would analyze joint velocities, currents, etc.
        pass
        
    def fault_injection_cycle(self):
        """🚨 SYSTEMATIC FAULT INJECTION"""
        
        current_time = time.time() - self.start_time
        
        # ═══ SCHEDULED FAULT INJECTION ═══
        for fault in self.fault_schedule:
            if not fault['triggered'] and current_time >= fault['time']:
                self.inject_fault(fault['component'], fault['severity'])
                fault['triggered'] = True
                
        # ═══ RANDOM FAULT INJECTION ═══
        if random.random() < 0.001:  # 0.1% chance per second
            available_components = [comp for comp in self.health_status.keys() 
                                  if self.health_status[comp]['status']]
            if available_components:
                component = random.choice(available_components)
                severity = random.uniform(0.2, 0.6)
                self.inject_fault(component, severity)
                
    def inject_fault(self, component, severity):
        """💥 INJECT SPECIFIC FAULT"""
        
        self.performance_metrics['total_faults'] += 1
        
        # Update component health
        self.health_status[component]['status'] = False
        self.health_status[component]['performance'] = 1.0 - severity
        
        # Start recovery process
        self.initiate_recovery(component, severity)
        
        # Log fault injection
        fault_msg = f"⚠️ FAULT INJECTED: {component.upper()} - Severity: {severity:.1%}"
        self.get_logger().error(fault_msg)
        
        # Determine recovery strategy based on component
        if component in ['left_wheel', 'right_wheel']:
            self.recovery_strategy = 'mobility_compensation'
            self.get_logger().info("🔧 STRATEGY: Mobility compensation activated")
        elif component == 'lidar':
            self.recovery_strategy = 'sensor_fallback'
            self.get_logger().info("🔧 STRATEGY: Sensor fallback activated")
        elif component == 'camera':
            self.recovery_strategy = 'vision_degraded'
            self.get_logger().info("🔧 STRATEGY: Vision degraded mode activated")
        else:
            self.recovery_strategy = 'general_degraded'
            self.get_logger().info("🔧 STRATEGY: General degraded operation")
            
    def initiate_recovery(self, component, severity):
        """🚀 INITIATE AUTONOMOUS RECOVERY"""
        
        if not self.in_recovery_mode:
            self.in_recovery_mode = True
            self.recovery_start_time = time.time()
            self.recovery_phase = 1
            self.recovery_target_component = component
            
            self.get_logger().warn(f"🔧 RECOVERY INITIATED: {component} failure detected")
            self.get_logger().info("📋 Starting multi-phase recovery protocol")
            
    def recovery_execution_cycle(self):
        """🔄 EXECUTE RECOVERY PHASES"""
        
        if not self.in_recovery_mode:
            return
            
        recovery_time = time.time() - self.recovery_start_time
        
        # ═══ PHASE-BASED RECOVERY ═══
        if self.recovery_strategy == 'mobility_compensation':
            self.execute_mobility_recovery(recovery_time)
        elif self.recovery_strategy == 'sensor_fallback':
            self.execute_sensor_recovery(recovery_time)
        elif self.recovery_strategy == 'vision_degraded':
            self.execute_vision_recovery(recovery_time)
        else:
            self.execute_general_recovery(recovery_time)
            
        # ═══ RECOVERY TIMEOUT ═══
        if recovery_time > 30.0:  # 30 second timeout
            self.recovery_timeout()
            
    def execute_mobility_recovery(self, recovery_time):
        """🦾 MOBILITY COMPENSATION RECOVERY"""
        
        cmd = Twist()
        
        if self.recovery_phase == 1 and recovery_time < 3.0:
            # Phase 1: Emergency stop and assessment
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().info("🔧 MOBILITY RECOVERY Phase 1: Emergency stop")
            
        elif self.recovery_phase == 1:
            self.recovery_phase = 2
            self.get_logger().info("🔧 MOBILITY RECOVERY Phase 2: Compensated movement")
            
        if self.recovery_phase == 2 and recovery_time < 12.0:
            # Phase 2: Compensated movement
            if self.recovery_target_component == 'left_wheel':
                # Compensate for left wheel failure
                cmd.linear.x = 0.15  # Reduced speed
                cmd.angular.z = 0.3   # Right turn bias to compensate
            elif self.recovery_target_component == 'right_wheel':
                # Compensate for right wheel failure
                cmd.linear.x = 0.15  # Reduced speed
                cmd.angular.z = -0.3  # Left turn bias to compensate
                
        elif self.recovery_phase == 2:
            self.recovery_phase = 3
            self.get_logger().info("🔧 MOBILITY RECOVERY Phase 3: Attempting self-repair")
            
        if self.recovery_phase == 3 and recovery_time < 18.0:
            # Phase 3: Attempt self-repair (vibration, system reset)
            cmd.linear.x = 0.1 if int(recovery_time * 2) % 2 else -0.1  # Vibration
            cmd.angular.z = 0.2 if int(recovery_time * 3) % 2 else -0.2
            
        elif self.recovery_phase == 3:
            # Attempt recovery completion
            if random.random() > 0.3:  # 70% success rate
                self.complete_successful_recovery()
            else:
                self.recovery_phase = 4
                
        if self.recovery_phase == 4:
            # Phase 4: Degraded operation mode
            cmd.linear.x = 0.05  # Very slow operation
            cmd.angular.z = 0.0
            self.get_logger().info("🔧 MOBILITY RECOVERY Phase 4: Degraded operation")
            
        self.cmd_pub.publish(cmd)
        
    def execute_sensor_recovery(self, recovery_time):
        """📡 SENSOR FALLBACK RECOVERY"""
        
        if self.recovery_phase == 1 and recovery_time < 2.0:
            self.get_logger().info("🔧 SENSOR RECOVERY Phase 1: Switching to backup sensors")
            
        elif self.recovery_phase == 1:
            self.recovery_phase = 2
            
        if self.recovery_phase == 2 and recovery_time < 8.0:
            self.get_logger().info("🔧 SENSOR RECOVERY Phase 2: Recalibrating sensor arrays")
            
        elif self.recovery_phase == 2:
            self.recovery_phase = 3
            
        if self.recovery_phase == 3:
            # Attempt sensor recovery
            if random.random() > 0.4:  # 60% success rate
                self.complete_successful_recovery()
            else:
                self.get_logger().warn("🔧 SENSOR RECOVERY: Using degraded sensor mode")
                self.complete_partial_recovery()
                
    def execute_vision_recovery(self, recovery_time):
        """👁️ VISION SYSTEM RECOVERY"""
        
        if recovery_time < 5.0:
            self.get_logger().info("🔧 VISION RECOVERY: Switching to LiDAR-only navigation")
        else:
            if random.random() > 0.5:  # 50% success rate
                self.complete_successful_recovery()
            else:
                self.complete_partial_recovery()
                
    def execute_general_recovery(self, recovery_time):
        """🔄 GENERAL RECOVERY PROTOCOL"""
        
        if recovery_time < 10.0:
            self.get_logger().info("🔧 GENERAL RECOVERY: Attempting system restoration")
        else:
            self.complete_partial_recovery()
            
    def execute_obstacle_recovery(self, scan_msg):
        """🚧 OBSTACLE-AWARE RECOVERY"""
        
        # Simple obstacle avoidance during recovery
        front_ranges = scan_msg.ranges[len(scan_msg.ranges)//4:3*len(scan_msg.ranges)//4]
        min_distance = min(front_ranges) if front_ranges else float('inf')
        
        cmd = Twist()
        if min_distance < 1.0:  # Obstacle too close
            cmd.linear.x = -0.1  # Back up
            cmd.angular.z = 0.5   # Turn away
            self.cmd_pub.publish(cmd)
            
    def check_stuck_condition(self):
        """🚫 DETECT STUCK CONDITION"""
        
        if len(self.last_positions) < self.position_history_size:
            return
            
        # Calculate movement in recent history
        recent_positions = self.last_positions[-15:]  # Last 3 seconds
        
        if len(recent_positions) >= 2:
            start_pos = recent_positions[0]
            end_pos = recent_positions[-1]
            
            distance_moved = math.sqrt(
                (end_pos[0] - start_pos[0])**2 + 
                (end_pos[1] - start_pos[1])**2
            )
            
            if distance_moved < self.stuck_threshold and not self.is_stuck:
                self.is_stuck = True
                self.inject_fault('mobility', 0.8)  # Treat stuck as mobility fault
                self.get_logger().warn("🚫 STUCK DETECTED: Initiating stuck recovery protocol")
                
    def complete_successful_recovery(self):
        """✅ COMPLETE SUCCESSFUL RECOVERY"""
        
        recovery_time = time.time() - self.recovery_start_time
        
        # Restore component health
        self.health_status[self.recovery_target_component]['status'] = True
        self.health_status[self.recovery_target_component]['performance'] = 1.0
        
        # Update metrics
        self.performance_metrics['successful_recoveries'] += 1
        current_avg = self.performance_metrics['recovery_time_avg']
        successful_count = self.performance_metrics['successful_recoveries']
        self.performance_metrics['recovery_time_avg'] = (current_avg * (successful_count - 1) + recovery_time) / successful_count
        
        # Reset recovery state
        self.in_recovery_mode = False
        self.recovery_phase = 0
        self.is_stuck = False
        
        self.get_logger().info(f"✅ RECOVERY SUCCESSFUL: {self.recovery_target_component} fully restored!")
        self.get_logger().info(f"⏱️ Recovery completed in {recovery_time:.1f} seconds")
        
    def complete_partial_recovery(self):
        """⚠️ COMPLETE PARTIAL RECOVERY"""
        
        recovery_time = time.time() - self.recovery_start_time
        
        # Partial restore
        self.health_status[self.recovery_target_component]['status'] = True
        self.health_status[self.recovery_target_component]['performance'] = 0.6  # Degraded performance
        
        # Reset recovery state
        self.in_recovery_mode = False
        self.recovery_phase = 0
        
        self.get_logger().warn(f"⚠️ PARTIAL RECOVERY: {self.recovery_target_component} operating in degraded mode")
        self.get_logger().info(f"⏱️ Recovery completed in {recovery_time:.1f} seconds")
        
    def recovery_timeout(self):
        """⏰ HANDLE RECOVERY TIMEOUT"""
        
        self.performance_metrics['failed_recoveries'] += 1
        
        # Emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        self.in_recovery_mode = False
        self.recovery_phase = 0
        
        self.get_logger().error("🚨 RECOVERY TIMEOUT: Emergency stop activated")
        self.get_logger().error("📞 Manual intervention required")
        
    def health_monitoring_cycle(self):
        """💓 CONTINUOUS HEALTH MONITORING"""
        
        # ═══ CALCULATE OVERALL SYSTEM HEALTH ═══
        total_health = sum(comp['performance'] for comp in self.health_status.values())
        avg_health = total_health / len(self.health_status) * 100
        
        active_faults = sum(1 for comp in self.health_status.values() if not comp['status'])
        
        # ═══ PUBLISH HEALTH STATUS ═══
        health_summary = []
        for component, status in self.health_status.items():
            icon = "✅" if status['status'] else "❌"
            perf = status['performance'] * 100
            health_summary.append(f"{component}:{icon}({perf:.0f}%)")
            
        health_msg = String()
        health_msg.data = f"HEALTH:{avg_health:.0f}% | FAULTS:{active_faults} | " + " | ".join(health_summary[:3])
        self.health_pub.publish(health_msg)
        
        # ═══ PUBLISH RECOVERY STATUS ═══
        recovery_msg = Bool()
        recovery_msg.data = self.in_recovery_mode
        self.recovery_pub.publish(recovery_msg)
        
        # ═══ PUBLISH PERFORMANCE METRICS ═══
        metrics_text = (
            f"FAULTS:{self.performance_metrics['total_faults']} | "
            f"RECOVERED:{self.performance_metrics['successful_recoveries']} | "
            f"FAILED:{self.performance_metrics['failed_recoveries']} | "
            f"AVG_RECOVERY:{self.performance_metrics['recovery_time_avg']:.1f}s"
        )
        
        metrics_msg = String()
        metrics_msg.data = metrics_text
        self.metrics_pub.publish(metrics_msg)
        
        # ═══ PUBLISH FAULT STATUS ═══
        status_parts = [f"HEALTH:{avg_health:.0f}%"]
        
        if self.in_recovery_mode:
            status_parts.append(f"RECOVERY:Phase{self.recovery_phase}")
            status_parts.append(f"TARGET:{self.recovery_target_component}")
            
        if active_faults > 0:
            status_parts.append(f"FAULTS:{active_faults}")
            
        fault_status_msg = String()
        fault_status_msg.data = " | ".join(status_parts)
        self.fault_status_pub.publish(fault_status_msg)

def main():
    rclpy.init()
    fault_system = FaultRecoverySystem()
    
    try:
        rclpy.spin(fault_system)
    except KeyboardInterrupt:
        fault_system.get_logger().info("🛑 Fault Recovery System stopped")
    finally:
        fault_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
