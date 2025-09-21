#!/usr/bin/env python3
import cv2
import numpy as np
import os

def fix_heightmap_size(input_image_path, output_heightmap_path):
    """Create properly sized heightmap for Gazebo"""
    
    print(f"🔧 Fixing heightmap size...")
    
    # Load original image
    original = cv2.imread(input_image_path, cv2.IMREAD_GRAYSCALE)
    if original is None:
        raise FileNotFoundError(f"Image not found: {input_image_path}")
    
    print(f"📏 Original image size: {original.shape}")
    
    # Resize to proper Gazebo heightmap size (513x513 = 2^9 + 1)
    target_size = 513  # 2^9 + 1 (valid Gazebo heightmap size)
    
    # Resize image to square
    heightmap_square = cv2.resize(original, (target_size, target_size))
    
    # Apply Gaussian blur for smooth terrain
    heightmap_smooth = cv2.GaussianBlur(heightmap_square, (5, 5), 0)
    
    print(f"✅ Resized to: {heightmap_smooth.shape}")
    
    # Create output directory
    os.makedirs(os.path.dirname(output_heightmap_path), exist_ok=True)
    
    # Save fixed heightmap
    cv2.imwrite(output_heightmap_path, heightmap_smooth)
    
    print(f"💾 Fixed heightmap saved: {output_heightmap_path}")
    
    return True

def create_fixed_world_file(world_path, heightmap_path, texture_path):
    """Create world file with correct paths"""
    
    print(f"🌍 Creating fixed world file...")
    
    world_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lunar_photorealistic">
    
    <!-- Lunar Physics -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -1.62</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>15</iters>
          <sor>1.3</sor>
        </solver>
      </ode>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Harsh lunar lighting -->
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>50 30 100 0 0 0</pose>
      <diffuse>1.0 1.0 0.95 1</diffuse>
      <specular>0.9 0.9 0.85 1</specular>
      <direction>-0.5 -0.3 -0.8</direction>
    </light>

    <!-- Space background -->
    <scene>
      <ambient>0.05 0.05 0.05 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- Fixed Lunar Terrain -->
    <model name="lunar_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_path}</uri>
              <size>400 400 50</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.05</mu>
                <mu2>0.05</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <geometry>
            <heightmap>
              <uri>{heightmap_path}</uri>
              <size>400 400 50</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>{texture_path}</diffuse>
                <size>20</size>
              </texture>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ROS Plugins -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>

    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so" />

  </world>
</sdf>'''
    
    # Create directory and write file
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    with open(world_path, 'w') as f:
        f.write(world_content)
    
    print(f"✅ Fixed world file created: {world_path}")

def main():
    """Fix heightmap and world file"""
    
    print("🔧 FIXING HEIGHTMAP SIZE ISSUE...")
    print("=" * 40)
    
    # Paths
    input_image = "/home/darshan/lunar/lunar_surface.jpg"
    heightmap_path = "/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap_fixed.png"
    texture_path = "/home/darshan/lunar/src/lunabot_simulation/media/materials/textures/lunar_surface.jpg"
    world_path = "/home/darshan/lunar/src/lunabot_simulation/worlds/lunar_fixed.world"
    
    try:
        # Fix heightmap size
        fix_heightmap_size(input_image, heightmap_path)
        
        # Create fixed world file
        create_fixed_world_file(world_path, heightmap_path, texture_path)
        
        print("=" * 40)
        print("🎉 HEIGHTMAP FIXED!")
        print("=" * 40)
        print(f"📁 Fixed heightmap: {heightmap_path}")
        print(f"🌍 Fixed world: {world_path}")
        print("=" * 40)
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
