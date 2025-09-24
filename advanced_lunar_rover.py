#!/usr/bin/env python3
import os

class AdvancedLunarRoverGenerator:
    """🤖 Advanced Lunar Rover with Multi-Sensor Fusion"""
    
    def __init__(self):
        self.lunar_path = "/home/darshan/lunar"
        self.urdf_dir = f"{self.lunar_path}/urdf"
        os.makedirs(self.urdf_dir, exist_ok=True)
        
    def create_advanced_rover_urdf(self):
        """Create advanced rover URDF with full sensor suite"""
        
        print("🤖 Creating advanced multi-sensor lunar rover...")
        
        rover_urdf = '''<?xml version="1.0"?>
<robot name="nasa_lunar_rover">

  <!-- Materials -->
  <material name="rover_grey">
    <color rgba="0.7 0.7 0.8 1"/>
  </material>
  
  <material name="rover_blue">
    <color rgba="0.3 0.4 0.8 1"/>
  </material>
  
  <material name="wheel_black">
    <color rgba="0.2 0.2 0.2 1"/>
  </material>
  
  <material name="sensor_red">
    <color rgba="0.8 0.2 0.2 1"/>
  </material>

  <!-- Main Chassis -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 1.5 0.4"/>
      </geometry>
      <material name="rover_grey"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="2.0 1.5 0.4"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <mass value="150"/>
      <inertia ixx="15.0" ixy="0" ixz="0" iyy="25.0" iyz="0" izz="35.0"/>
    </inertial>
  </link>

  <!-- Sensor Mast -->
  <link name="sensor_mast">
    <visual>
      <origin xyz="0 0 0.6" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="1.2"/>
      </geometry>
      <material name="rover_blue"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0.6" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.05" length="1.2"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0.6" rpy="0 0 0"/>
      <mass value="8"/>
      <inertia ixx="1.0" ixy="0" ixz="0" iyy="1.0" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="mast_joint" type="fixed">
    <parent link="base_link"/>
    <child link="sensor_mast"/>
    <origin xyz="0.5 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- LiDAR Sensor -->
  <link name="lidar_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.12"/>
      </geometry>
      <material name="sensor_red"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <cylinder radius="0.08" length="0.12"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="sensor_mast"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 1.1" rpy="0 0 0"/>
  </joint>

  <!-- RGB-D Camera -->
  <link name="camera_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.12 0.08 0.06"/>
      </geometry>
      <material name="rover_blue"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.12 0.08 0.06"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="sensor_mast"/>
    <child link="camera_link"/>
    <origin xyz="0.1 0 1.0" rpy="0 0.1 0"/>
  </joint>

  <!-- IMU Sensor -->
  <link name="imu_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.04 0.04 0.02"/>
      </geometry>
      <material name="sensor_red"/>
    </visual>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
  </joint>

  <!-- Wheels -->
  <link name="front_left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
      <material name="wheel_black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="12"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>

  <joint name="front_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_left_wheel"/>
    <origin xyz="0.8 0.9 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="front_right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
      <material name="wheel_black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="12"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>

  <joint name="front_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="front_right_wheel"/>
    <origin xyz="0.8 -0.9 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="rear_left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
      <material name="wheel_black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="12"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>

  <joint name="rear_left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_left_wheel"/>
    <origin xyz="-0.8 0.9 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <link name="rear_right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
      <material name="wheel_black"/>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.25" length="0.15"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="12"/>
      <inertia ixx="0.5" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.5"/>
    </inertial>
  </link>

  <joint name="rear_right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="rear_right_wheel"/>
    <origin xyz="-0.8 -0.9 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>

  <!-- Gazebo Plugins -->
  
  <!-- Differential Drive Controller -->
  <gazebo>
    <plugin name="skid_steer_drive_controller" filename="libgazebo_ros_skid_steer_drive.so">
      <update_rate>50.0</update_rate>
      <robot_namespace>/</robot_namespace>
      <left_front_joint>front_left_wheel_joint</left_front_joint>
      <right_front_joint>front_right_wheel_joint</right_front_joint>
      <left_rear_joint>rear_left_wheel_joint</left_rear_joint>
      <right_rear_joint>rear_right_wheel_joint</right_rear_joint>
      <wheel_separation>1.8</wheel_separation>
      <wheel_diameter>0.5</wheel_diameter>
      <robot_base_frame>base_link</robot_base_frame>
      <max_wheel_torque>500</max_wheel_torque>
      <max_wheel_acceleration>15.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </gazebo>

  <!-- LiDAR Plugin -->
  <gazebo reference="lidar_link">
    <sensor type="gpu_ray" name="lidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>20</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>720</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle>
            <max_angle>3.14159</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.10</min>
          <max>100.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_laser" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
      </plugin>
    </sensor>
  </gazebo>

  <!-- RGB-D Camera Plugin -->
  <gazebo reference="camera_link">
    <sensor type="depth" name="camera_sensor">
      <update_rate>20</update_rate>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.02</near>
          <far>300</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="gazebo_ros_depth_camera" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>~/image_raw:=camera/image_raw</remapping>
          <remapping>~/image_depth:=camera/depth/image_raw</remapping>
          <remapping>~/camera_info:=camera/camera_info</remapping>
          <remapping>~/camera_info_depth:=camera/depth/camera_info</remapping>
          <remapping>~/points:=camera/depth/points</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
        <hack_baseline>0.07</hack_baseline>
        <min_depth>0.001</min_depth>
        <max_depth>300.0</max_depth>
      </plugin>
    </sensor>
  </gazebo>

  <!-- IMU Plugin -->
  <gazebo reference="imu_link">
    <sensor name="imu_sensor" type="imu">
      <update_rate>100</update_rate>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <remapping>~/out:=imu</remapping>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
      </plugin>
    </sensor>
  </gazebo>

</robot>'''

        # Save URDF
        urdf_path = f"{self.urdf_dir}/nasa_lunar_rover.urdf"
        with open(urdf_path, 'w') as f:
            f.write(rover_urdf)
            
        print(f"✅ Advanced lunar rover URDF created: {urdf_path}")
        return urdf_path
        
    def run_generation(self):
        """Generate complete rover"""
        
        print("🤖 ADVANCED LUNAR ROVER GENERATION")
        print("=" * 50)
        
        urdf_path = self.create_advanced_rover_urdf()
        
        print("=" * 50)
        print("🎉 ROVER GENERATION COMPLETE!")
        print(f"🤖 URDF: {urdf_path}")
        print("=" * 50)

def main():
    generator = AdvancedLunarRoverGenerator()
    generator.run_generation()

if __name__ == "__main__":
    main()
