from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    # Get URDF file path
    urdf_file = os.path.join(os.getcwd(), 'src/lunabot_simulation/urdf/lunabot.urdf')
    
    # Start Gazebo with lunar world
    start_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', 'src/lunabot_simulation/worlds/lunar_habitat.world'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )
    
    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'lunabot', '-topic', '/robot_description', '-x', '0', '-y', '0', '-z', '0.1'],
        output='screen'
    )
    
    return LaunchDescription([
        start_gazebo,
        robot_state_publisher,
        spawn_robot
    ])

