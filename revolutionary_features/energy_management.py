#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Float32, String, Bool
import math
import time
import random
import numpy as np

class EnergyManagementSystem(Node):
    """⚡ Revolutionary Energy Management & Charging Behavior"""
    
    def __init__(self):
        super().__init__('energy_management_system')
        
        # ═══ BATTERY SIMULATION ═══
        self.battery_level = 100.0  # Start fully charged
        self.battery_capacity = 100.0  # 100% capacity
        self.battery_health = 1.0   # Battery health factor
        
        # ═══ ENERGY CONSUMPTION RATES ═══
        self.consumption_rates = {
            'movement': 0.08,      # %/meter moved
            'turning': 0.05,       # %/radian turned  
            'lidar': 0.02,         # %/second active
            'camera': 0.01,        # %/second active
            'computer': 0.03,      # %/second base consumption
            'dust_storm': 1.5,     # Multiplier during dust storms
            'recovery': 2.0,       # Multiplier during recovery
            'idle': 0.01          # %/second when idle
        }
        
        # ═══ SOLAR CHARGING SIMULATION ═══
        self.solar_panels = {
            'efficiency': 0.85,     # 85% efficient panels
            'area': 2.0,           # 2 m² of panels
            'max_power': 400,      # 400W maximum
            'current_power': 0.0,  # Current power generation
            'sun_angle': 0.0,      # Angle to sun
            'dust_degradation': 1.0  # Dust on panels reduces efficiency
        }
        
        # ═══ CHARGING STATIONS ═══
        self.charging_stations = [
            {'id': 'station_alpha', 'x': 20.0, 'y': 15.0, 'power': 1000, 'occupied': False},
            {'id': 'station_beta', 'x': -25.0, 'y': 30.0, 'power': 800, 'occupied': False},
            {'id': 'station_gamma', 'x': 10.0, 'y': -20.0, 'power': 1200, 'occupied': False}
        ]
        
        # ═══ ENERGY MANAGEMENT STATE ═══
        self.energy_mode = 'normal'  # normal, conservative, emergency, charging
        self.low_battery_threshold = 30.0
        self.critical_battery_threshold = 15.0
        self.charging_target = 85.0
        
        # ═══ CURRENT STATE TRACKING ═══
        self.current_position = None
        self.last_position = None
        self.is_moving = False
        self.target_charging_station = None
        self.at_charging_station = False
        self.charging_start_time = 0
        
        # ═══ ENVIRONMENTAL FACTORS ═══
        self.current_dust_level = 0.0
        self.in_dust_storm = False
        self.in_recovery_mode = False
        self.system_load_factor = 1.0
        
        # ═══ PERFORMANCE METRICS ═══
        self.energy_metrics = {
            'total_energy_consumed': 0.0,
            'energy_from_solar': 0.0,
            'energy_from_stations': 0.0,
            'charging_cycles': 0,
            'energy_efficiency': 0.0,
            'distance_per_charge': 0.0,
            'low_battery_events': 0
        }
        
        # ═══ ROS SUBSCRIBERS ═══
        self.pose_sub = self.create_subscription(PoseStamped, '/adaptive_slam_pose', self.pose_callback, 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.command_callback, 10)
        self.dust_sub = self.create_subscription(Float32, '/dust_level', self.dust_callback, 10)
        self.storm_sub = self.create_subscription(Bool, '/dust_storm_active', self.storm_callback, 10)
        self.recovery_sub = self.create_subscription(Bool, '/recovery_mode_active', self.recovery_callback, 10)
        
        # ═══ ROS PUBLISHERS ═══
        self.battery_pub = self.create_publisher(Float32, '/battery_level', 10)
        self.energy_status_pub = self.create_publisher(String, '/energy_status', 10)
        self.charging_pub = self.create_publisher(Bool, '/charging_active', 10)
        self.solar_pub = self.create_publisher(Float32, '/solar_power', 10)
        self.energy_mode_pub = self.create_publisher(String, '/energy_mode', 10)
        self.station_path_pub = self.create_publisher(Path, '/charging_station_path', 10)
        self.energy_cmd_pub = self.create_publisher(Twist, '/energy_aware_cmd_vel', 10)
        
        # ═══ TIMERS ═══
        self.energy_simulation_timer = self.create_timer(0.5, self.energy_simulation_cycle)
        self.solar_simulation_timer = self.create_timer(2.0, self.solar_simulation_cycle)
        self.energy_management_timer = self.create_timer(1.0, self.energy_management_cycle)
        self.charging_timer = self.create_timer(0.2, self.charging_cycle)
        
        self.start_time = time.time()
        
        self.get_logger().info("⚡ ENERGY MANAGEMENT: Revolutionary energy system started!")
        self.get_logger().info("🔋 Features: Smart consumption, Solar charging, Station navigation, Adaptive planning")
        
    def pose_callback(self, msg):
        """Track rover position for energy calculations"""
        
        self.last_position = self.current_position
        self.current_position = msg.pose.position
        
        # Check if at charging station
        self.check_charging_station_proximity()
        
    def command_callback(self, msg):
        """Monitor movement commands for energy consumption"""
        
        # Detect movement
        self.is_moving = (msg.linear.x != 0.0 or msg.angular.z != 0.0)
        
        if self.is_moving:
            # Apply energy-aware modifications to commands
            modified_cmd = self.apply_energy_optimization(msg)
            self.energy_cmd_pub.publish(modified_cmd)
            
    def dust_callback(self, msg):
        """Monitor dust level affecting solar panels"""
        
        self.current_dust_level = msg.data
        
        # Dust reduces solar panel efficiency
        self.solar_panels['dust_degradation'] = 1.0 - (self.current_dust_level * 0.6)  # Up to 60% reduction
        
    def storm_callback(self, msg):
        """Monitor dust storm affecting energy consumption"""
        
        self.in_dust_storm = msg.data
        
    def recovery_callback(self, msg):
        """Monitor recovery mode affecting energy consumption"""
        
        self.in_recovery_mode = msg.data
        
    def energy_simulation_cycle(self):
        """🔋 MAIN ENERGY SIMULATION CYCLE"""
        
        # ═══ CALCULATE ENERGY CONSUMPTION ═══
        energy_consumed = self.calculate_energy_consumption()
        
        # ═══ UPDATE BATTERY LEVEL ═══
        self.battery_level = max(0.0, self.battery_level - energy_consumed)
        
        # ═══ UPDATE METRICS ═══
        self.energy_metrics['total_energy_consumed'] += energy_consumed
        
        # ═══ CALCULATE EFFICIENCY ═══
        if self.last_position and self.current_position:
            distance = math.sqrt(
                (self.current_position.x - self.last_position.x)**2 + 
                (self.current_position.y - self.last_position.y)**2
            )
            if energy_consumed > 0:
                efficiency = distance / energy_consumed
                self.energy_metrics['energy_efficiency'] = efficiency
                
        # ═══ PUBLISH BATTERY STATUS ═══
        battery_msg = Float32()
        battery_msg.data = self.battery_level
        self.battery_pub.publish(battery_msg)
        
    def calculate_energy_consumption(self):
        """Calculate energy consumption based on current activities"""
        
        base_consumption = self.consumption_rates['computer'] * 0.5  # 0.5 second interval
        total_consumption = base_consumption
        
        # ═══ MOVEMENT ENERGY ═══
        if self.is_moving and self.last_position and self.current_position:
            distance = math.sqrt(
                (self.current_position.x - self.last_position.x)**2 + 
                (self.current_position.y - self.last_position.y)**2
            )
            movement_energy = distance * self.consumption_rates['movement']
            total_consumption += movement_energy
            
        # ═══ SENSOR ENERGY ═══
        total_consumption += self.consumption_rates['lidar'] * 0.5
        total_consumption += self.consumption_rates['camera'] * 0.5
        
        # ═══ ENVIRONMENTAL MULTIPLIERS ═══
        if self.in_dust_storm:
            total_consumption *= self.consumption_rates['dust_storm']
            
        if self.in_recovery_mode:
            total_consumption *= self.consumption_rates['recovery']
            
        # ═══ SYSTEM LOAD FACTOR ═══
        total_consumption *= self.system_load_factor
        
        return total_consumption
        
    def solar_simulation_cycle(self):
        """☀️ SOLAR PANEL SIMULATION"""
        
        current_time = time.time() - self.start_time
        
        # ═══ SIMULATE SUN ANGLE ═══
        # Simple day/night cycle (12 hour period for demo)
        day_progress = (current_time % 720) / 720  # 12 minute day cycle
        sun_elevation = math.sin(day_progress * 2 * math.pi) * 90  # -90 to +90 degrees
        
        # ═══ CALCULATE SOLAR POWER ═══
        if sun_elevation > 0:  # Daytime
            # Solar power based on sun angle and panel efficiency
            angle_factor = math.sin(math.radians(sun_elevation))
            base_power = self.solar_panels['max_power'] * angle_factor
            
            # Apply efficiency and dust degradation
            actual_power = (base_power * 
                          self.solar_panels['efficiency'] * 
                          self.solar_panels['dust_degradation'])
            
            self.solar_panels['current_power'] = actual_power
            
            # Convert to battery percentage (simplified)
            energy_gain = (actual_power / 1000) * (2.0 / 3600) * 100  # Rough conversion
            self.battery_level = min(self.battery_capacity, self.battery_level + energy_gain)
            self.energy_metrics['energy_from_solar'] += energy_gain
            
        else:  # Nighttime
            self.solar_panels['current_power'] = 0.0
            
        # ═══ PUBLISH SOLAR STATUS ═══
        solar_msg = Float32()
        solar_msg.data = self.solar_panels['current_power']
        self.solar_pub.publish(solar_msg)
        
    def energy_management_cycle(self):
        """🧠 INTELLIGENT ENERGY MANAGEMENT"""
        
        # ═══ DETERMINE ENERGY MODE ═══
        if self.battery_level <= self.critical_battery_threshold:
            self.energy_mode = 'emergency'
        elif self.battery_level <= self.low_battery_threshold:
            if not self.at_charging_station:
                self.energy_mode = 'seeking_charge'
            else:
                self.energy_mode = 'charging'
        elif self.at_charging_station and self.battery_level < self.charging_target:
            self.energy_mode = 'charging'
        else:
            self.energy_mode = 'normal'
            
        # ═══ EXECUTE ENERGY MANAGEMENT STRATEGY ═══
        if self.energy_mode == 'emergency':
            self.execute_emergency_protocol()
        elif self.energy_mode == 'seeking_charge':
            self.execute_charging_navigation()
        elif self.energy_mode == 'charging':
            self.execute_charging_protocol()
        elif self.energy_mode == 'normal':
            self.execute_normal_operation()
            
        # ═══ PUBLISH ENERGY STATUS ═══
        self.publish_energy_status()
        
    def execute_emergency_protocol(self):
        """🚨 EMERGENCY LOW BATTERY PROTOCOL"""
        
        self.get_logger().error("🚨 EMERGENCY: Critical battery level!")
        
        # Stop all non-essential systems
        self.system_load_factor = 0.3  # Minimum power mode
        
        # Find nearest charging station
        if not self.target_charging_station:
            self.target_charging_station = self.find_nearest_charging_station()
            
        if self.target_charging_station:
            self.get_logger().warn(f"🔋 EMERGENCY CHARGE: Heading to {self.target_charging_station['id']}")
        else:
            self.get_logger().error("🔋 NO CHARGING STATIONS AVAILABLE!")
            
    def execute_charging_navigation(self):
        """🎯 NAVIGATE TO CHARGING STATION"""
        
        if not self.target_charging_station:
            self.target_charging_station = self.find_nearest_charging_station()
            self.energy_metrics['low_battery_events'] += 1
            
        if self.target_charging_station and self.current_position:
            # Calculate path to charging station
            station_pos = self.target_charging_station
            
            distance_to_station = math.sqrt(
                (station_pos['x'] - self.current_position.x)**2 + 
                (station_pos['y'] - self.current_position.y)**2
            )
            
            if distance_to_station < 2.0:  # Within 2 meters
                self.get_logger().info(f"🔌 ARRIVED: At {station_pos['id']}")
                self.at_charging_station = True
                station_pos['occupied'] = True
                
            # Publish path visualization
            self.publish_charging_station_path()
            
    def execute_charging_protocol(self):
        """🔌 CHARGING STATION PROTOCOL"""
        
        if not self.at_charging_station:
            return
            
        if self.charging_start_time == 0:
            self.charging_start_time = time.time()
            self.get_logger().info("🔌 CHARGING STARTED: Fast charging initiated")
            
        # ═══ FAST CHARGING SIMULATION ═══
        if self.target_charging_station:
            charging_power = self.target_charging_station['power']  # Watts
            
            # Convert to battery percentage gain per second
            charge_rate = (charging_power / 1000) * (1.0 / 3600) * 100  # Simplified conversion
            
            self.battery_level = min(self.battery_capacity, self.battery_level + charge_rate * 1.0)  # 1 second
            self.energy_metrics['energy_from_stations'] += charge_rate
            
        # ═══ CHARGING COMPLETION ═══
        if self.battery_level >= self.charging_target:
            self.complete_charging()
            
        # ═══ PUBLISH CHARGING STATUS ═══
        charging_msg = Bool()
        charging_msg.data = True
        self.charging_pub.publish(charging_msg)
        
    def execute_normal_operation(self):
        """🟢 NORMAL ENERGY OPERATION"""
        
        # Normal system load
        self.system_load_factor = 1.0
        
        # Monitor for proactive charging opportunities
        if (self.battery_level < 60.0 and 
            self.solar_panels['current_power'] < 100 and  # Low solar
            self.find_nearest_charging_station_distance() < 30.0):  # Station nearby
            
            self.get_logger().info("⚡ PROACTIVE CHARGING: Opportunistic charge available")
            self.energy_mode = 'seeking_charge'
            
    def complete_charging(self):
        """✅ COMPLETE CHARGING SEQUENCE"""
        
        charging_duration = time.time() - self.charging_start_time
        
        self.get_logger().info(f"✅ CHARGING COMPLETE: Battery at {self.battery_level:.1f}%")
        self.get_logger().info(f"⏱️ Charging time: {charging_duration:.1f} seconds")
        
        # Reset charging state
        self.at_charging_station = False
        if self.target_charging_station:
            self.target_charging_station['occupied'] = False
            
        self.target_charging_station = None
        self.charging_start_time = 0
        self.energy_metrics['charging_cycles'] += 1
        
        # Calculate distance per charge cycle
        if self.energy_metrics['charging_cycles'] > 0:
            total_distance = self.energy_metrics['total_energy_consumed'] / self.consumption_rates['movement']
            self.energy_metrics['distance_per_charge'] = total_distance / self.energy_metrics['charging_cycles']
            
    def find_nearest_charging_station(self):
        """🔍 FIND NEAREST AVAILABLE CHARGING STATION"""
        
        if not self.current_position:
            return None
            
        available_stations = [s for s in self.charging_stations if not s['occupied']]
        
        if not available_stations:
            return None
            
        # Find closest station
        nearest_station = min(available_stations, key=lambda s: math.sqrt(
            (s['x'] - self.current_position.x)**2 + 
            (s['y'] - self.current_position.y)**2
        ))
        
        return nearest_station
        
    def find_nearest_charging_station_distance(self):
        """📏 GET DISTANCE TO NEAREST CHARGING STATION"""
        
        nearest = self.find_nearest_charging_station()
        if not nearest or not self.current_position:
            return float('inf')
            
        return math.sqrt(
            (nearest['x'] - self.current_position.x)**2 + 
            (nearest['y'] - self.current_position.y)**2
        )
        
    def check_charging_station_proximity(self):
        """📍 CHECK IF NEAR CHARGING STATION"""
        
        if not self.current_position:
            return
            
        for station in self.charging_stations:
            distance = math.sqrt(
                (station['x'] - self.current_position.x)**2 + 
                (station['y'] - self.current_position.y)**2
            )
            
            if distance < 2.0 and self.target_charging_station == station:
                if not self.at_charging_station:
                    self.at_charging_station = True
                    station['occupied'] = True
                break
        else:
            self.at_charging_station = False
            
    def apply_energy_optimization(self, cmd):
        """⚡ APPLY ENERGY-AWARE COMMAND MODIFICATIONS"""
        
        modified_cmd = Twist()
        modified_cmd.linear = cmd.linear
        modified_cmd.angular = cmd.angular
        
        # ═══ ENERGY MODE MODIFICATIONS ═══
        if self.energy_mode == 'emergency':
            # Very slow movement to conserve energy
            modified_cmd.linear.x *= 0.3
            modified_cmd.angular.z *= 0.3
            
        elif self.energy_mode == 'seeking_charge':
            # Moderate speed toward charging station
            modified_cmd.linear.x *= 0.6
            modified_cmd.angular.z *= 0.6
            
        elif self.battery_level < 50.0:
            # Conservative operation
            modified_cmd.linear.x *= 0.8
            modified_cmd.angular.z *= 0.8
            
        return modified_cmd
        
    def publish_charging_station_path(self):
        """🗺️ PUBLISH PATH TO CHARGING STATION"""
        
        if not self.target_charging_station or not self.current_position:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Simple straight-line path for visualization
        start_pose = PoseStamped()
        start_pose.header.frame_id = "map"
        start_pose.pose.position = self.current_position
        path_msg.poses.append(start_pose)
        
        end_pose = PoseStamped()
        end_pose.header.frame_id = "map"
        end_pose.pose.position.x = self.target_charging_station['x']
        end_pose.pose.position.y = self.target_charging_station['y']
        path_msg.poses.append(end_pose)
        
        self.station_path_pub.publish(path_msg)
        
    def charging_cycle(self):
        """🔄 HIGH-FREQUENCY CHARGING MONITORING"""
        
        if self.at_charging_station:
            # Publish active charging status
            charging_msg = Bool()
            charging_msg.data = True
            self.charging_pub.publish(charging_msg)
        else:
            charging_msg = Bool()
            charging_msg.data = False
            self.charging_pub.publish(charging_msg)
            
    def publish_energy_status(self):
        """📊 PUBLISH COMPREHENSIVE ENERGY STATUS"""
        
        # ═══ ENERGY STATUS MESSAGE ═══
        status_parts = [
            f"BATTERY:{self.battery_level:.1f}%",
            f"MODE:{self.energy_mode.upper()}",
            f"SOLAR:{self.solar_panels['current_power']:.0f}W"
        ]
        
        if self.at_charging_station:
            status_parts.append("🔌CHARGING")
        elif self.target_charging_station:
            status_parts.append(f"➡️{self.target_charging_station['id']}")
            
        if self.energy_mode == 'emergency':
            status_parts.append("🚨EMERGENCY")
            
        energy_status_msg = String()
        energy_status_msg.data = " | ".join(status_parts)
        self.energy_status_pub.publish(energy_status_msg)
        
        # ═══ ENERGY MODE MESSAGE ═══
        mode_msg = String()
        mode_msg.data = self.energy_mode
        self.energy_mode_pub.publish(mode_msg)

def main():
    rclpy.init()
    energy_system = EnergyManagementSystem()
    
    try:
        rclpy.spin(energy_system)
    except KeyboardInterrupt:
        energy_system.get_logger().info("⚡ Energy Management System stopped")
    finally:
        energy_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
