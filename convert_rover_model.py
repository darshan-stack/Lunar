#!/usr/bin/env python3
import os
import shutil
import subprocess

class RoverModelConverter:
    """Convert GLB/USDZ rover model to Gazebo-compatible format"""
    
    def __init__(self):
        self.lunar_path = "/home/darshan/lunar"
        self.models_dir = f"{self.lunar_path}/src/lunabot_simulation/models"
        
    def setup_model_directory(self):
        """Create model directory structure"""
        
        custom_rover_dir = f"{self.models_dir}/custom_rover"
        os.makedirs(custom_rover_dir, exist_ok=True)
        os.makedirs(f"{custom_rover_dir}/meshes", exist_ok=True)
        os.makedirs(f"{custom_rover_dir}/materials/textures", exist_ok=True)
        
        return custom_rover_dir
        
    def convert_rover_model(self, rover_model_path):
        """Convert rover model to Gazebo format"""
        
        print(f"🤖 Converting rover model: {rover_model_path}")
        
        custom_rover_dir = self.setup_model_directory()
        
        if rover_model_path.endswith('.glb'):
            # Convert GLB to OBJ using Blender
            self.convert_glb_to_obj(rover_model_path, custom_rover_dir)
        elif rover_model_path.endswith('.usdz'):
            # Handle USDZ format
            self.convert_usdz_to_obj(rover_model_path, custom_rover_dir)
        
        # Create model configuration
        self.create_rover_model_config(custom_rover_dir)
        
        # Create URDF for the custom rover
        self.create_custom_rover_urdf(custom_rover_dir)
        
        return custom_rover_dir
        
    def convert_glb_to_obj(self, glb_path, output_dir):
        """Convert GLB to OBJ using Blender"""
        
        print("🔄 Converting GLB to OBJ...")
        
        # Create Blender script for conversion
        blender_script = f'''
import bpy
import bmesh

# Clear existing mesh objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Import GLB
bpy.ops.import_scene.gltf(filepath="{glb_path}")

# Get all mesh objects
mesh_objects = [obj for obj in bpy.context.scene.objects if obj.type == 'MESH']

if mesh_objects:
    # Select all mesh objects
    for obj in mesh_objects:
        obj.select_set(True)
    
    bpy.context.view_layer.objects.active = mesh_objects[0]
    
    # Scale appropriately for Gazebo (typical rover size: 2m x 1.5m x 1m)
    bpy.ops.transform.resize(value=(0.5, 0.5, 0.5))
    
    # Export as OBJ
    bpy.ops.export_scene.obj(
        filepath="{output_dir}/meshes/rover.obj",
        use_selection=True,
        use_materials=True,
        use_smooth_groups=True,
        use_normals=True
    )
    
    print("✅ Rover model converted to OBJ")
else:
    print("❌ No mesh objects found in GLB file")
'''
        
        # Write script to file
        script_path = f"{output_dir}/convert_script.py"
        with open(script_path, 'w') as f:
            f.write(blender_script)
        
        # Try to run Blender conversion
        try:
            subprocess.run(['blender', '--background', '--python', script_path], 
                          check=True, capture_output=True)
            print("✅ GLB converted to OBJ successfully")
        except subprocess.CalledProcessError:
            print("⚠️ Blender conversion failed, creating fallback rover model...")
            self.create_fallback_rover_model(output_dir)
        except FileNotFoundError:
            print("⚠️ Blender not found, creating fallback rover model...")
            self.create_fallback_rover_model(output_dir)
            
    def convert_usdz_to_obj(self, usdz_path, output_dir):
        """Convert USDZ to OBJ (fallback to procedural model)"""
        
        print("⚠️ USDZ conversion requires specialized tools, creating custom rover model...")
        self.create_fallback_rover_model(output_dir)
        
    def create_fallback_rover_model(self, output_dir):
        """Create a custom procedural rover model"""
        
        print("🛠️ Creating custom procedural rover model...")
        
        # Create OBJ file with rover-like geometry
        rover_obj = '''# Custom Rover Model
# Vertices for rover body, wheels, and components
v -1.0 -0.75 0.0
v 1.0 -0.75 0.0
v 1.0 0.75 0.0
v -1.0 0.75 0.0
v -1.0 -0.75 0.5
v 1.0 -0.75 0.5
v 1.0 0.75 0.5
v -1.0 0.75 0.5

# Rover body faces
f 1 2 3 4
f 5 8 7 6
f 1 5 6 2
f 2 6 7 3
f 3 7 8 4
f 4 8 5 1

# Front wheels
v -1.2 -0.9 -0.2
v -1.2 -0.6 -0.2
v -1.2 -0.6 0.2
v -1.2 -0.9 0.2

f 9 10 11 12

v -1.2 0.6 -0.2
v -1.2 0.9 -0.2
v -1.2 0.9 0.2
v -1.2 0.6 0.2

f 13 14 15 16

# Rear wheels
v 1.2 -0.9 -0.2
v 1.2 -0.6 -0.2
v 1.2 -0.6 0.2
v 1.2 -0.9 0.2

f 17 18 19 20

v 1.2 0.6 -0.2
v 1.2 0.9 -0.2
v 1.2 0.9 0.2
v 1.2 0.6 0.2

f 21 22 23 24
'''
        
        # Save OBJ file
        obj_path = f"{output_dir}/meshes/rover.obj"
        with open(obj_path, 'w') as f:
            f.write(rover_obj)
        
        print(f"✅ Custom rover model created: {obj_path}")
        
    def create_rover_model_config(self, rover_dir):
        """Create Gazebo model configuration"""
        
        print("📋 Creating rover model configuration...")
        
        model_config = '''<?xml version="1.0"?>
<model>
  <name>custom_rover</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>Lunar AI Team</name>
    <email>team@lunar.ai</email>
  </author>
  <description>
    Custom 3D rover model for lunar exploration simulation
  </description>
</model>'''
        
        config_path = f"{rover_dir}/model.config"
        with open(config_path, 'w') as f:
            f.write(model_config)
        
        # Create SDF model file
        model_sdf = f'''<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="custom_rover">
    <static>false</static>
    
    <link name="chassis">
      <pose>0 0 0.5 0 0 0</pose>
      
      <inertial>
        <mass>50.0</mass>
        <inertia>
          <ixx>4.0</ixx>
          <iyy>8.0</iyy>
          <izz>10.0</izz>
        </inertia>
      </inertial>
      
      <collision name="collision">
        <geometry>
          <mesh>
            <uri>file://meshes/rover.obj</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </collision>
      
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>file://meshes/rover.obj</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>0.7 0.7 0.8 1</ambient>
          <diffuse>0.9 0.9 1.0 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
    </link>
    
    <!-- Wheels and joints would go here for full functionality -->
    
  </model>
</sdf>'''
        
        sdf_path = f"{rover_dir}/model.sdf"
        with open(sdf_path, 'w') as f:
            f.write(model_sdf)
        
        print(f"✅ Model configuration created: {config_path}")
        
    def create_custom_rover_urdf(self, rover_dir):
        """Create URDF for custom rover"""
        
        print("🔧 Creating custom rover URDF...")
        
        custom_urdf = f'''<?xml version="1.0"?>
<robot name="custom_rover">
  
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file://{rover_dir}/meshes/rover.obj" scale="1 1 1"/>
      </geometry>
      <material name="rover_material">
        <color rgba="0.8 0.8 0.9 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="file://{rover_dir}/meshes/rover.obj" scale="1 1 1"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="50"/>
      <inertia ixx="4.0" ixy="0" ixz="0" iyy="8.0" iyz="0" izz="10.0"/>
    </inertial>
  </link>
  
  <!-- Left Wheel -->
  <link name="left_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Right Wheel -->
  <link name="right_wheel">
    <visual>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
      <material name="wheel_material">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    
    <collision>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <geometry>
        <cylinder radius="0.2" length="0.1"/>
      </geometry>
    </collision>
    
    <inertial>
      <origin xyz="0 0 0" rpy="1.5708 0 0"/>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  
  <!-- Wheel Joints -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.25 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
  
  <!-- Gazebo Plugins -->
  <gazebo>
    <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
      <update_rate>30</update_rate>
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.5</wheel_separation>
      <wheel_diameter>0.4</wheel_diameter>
      <max_wheel_torque>200</max_wheel_torque>
      <max_wheel_acceleration>10.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>true</publish_wheel_tf>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
    </plugin>
  </gazebo>
  
</robot>'''
        
        urdf_path = f"{self.lunar_path}/src/lunabot_simulation/urdf/custom_rover.urdf"
        os.makedirs(os.path.dirname(urdf_path), exist_ok=True)
        
        with open(urdf_path, 'w') as f:
            f.write(custom_urdf)
        
        print(f"✅ Custom rover URDF created: {urdf_path}")
        return urdf_path

def main():
    converter = RoverModelConverter()
    
    print("🤖 ROVER MODEL CONVERSION STARTED")
    print("=" * 50)
    
    # Look for rover model files
    lunar_path = "/home/darshan/lunar"
    rover_files = []
    
    for file in os.listdir(lunar_path):
        if file.endswith(('.glb', '.usdz')):
            rover_files.append(os.path.join(lunar_path, file))
    
    if rover_files:
        print(f"📁 Found rover model: {rover_files[0]}")
        rover_dir = converter.convert_rover_model(rover_files[0])
    else:
        print("📁 No GLB/USDZ files found, creating custom rover...")
        rover_dir = converter.setup_model_directory()
        converter.create_fallback_rover_model(rover_dir)
        converter.create_rover_model_config(rover_dir)
        converter.create_custom_rover_urdf(rover_dir)
    
    print("=" * 50)
    print("🎉 ROVER MODEL CONVERSION COMPLETE!")
    print("=" * 50)
    print(f"🤖 Custom rover directory: {rover_dir}")
    print("✅ Ready for enhanced lunar simulation!")
    print("=" * 50)

if __name__ == "__main__":
    main()
