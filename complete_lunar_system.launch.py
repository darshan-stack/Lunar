from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        
        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
                 'src/lunabot_simulation/worlds/authentic_lunar_habitat.world'],
            output='screen'
        ),
        
        # AMFS-SLAM
        Node(
            package='amfs_slam',
            executable='simple_adaptive_slam_node',
            name='amfs_slam',
            output='screen'
        ),
        
        # Firefly Swarm Coordination
        Node(
            package='swarm_coordination', 
            executable='firefly_swarm_node',
            name='swarm_coordinator',
            output='screen'
        ),
        
        # Predictive Hazard Detection
        Node(
            package='predictive_hazard',
            executable='hazard_prediction_node', 
            name='hazard_predictor',
            output='screen'
        ),
        
    ])
