#!/usr/bin/env python3
import cv2
import numpy as np
import os

class AdvancedLunarSurfaceEnhancer:
    """Create ultra-realistic lunar surface"""
    
    def __init__(self):
        self.lunar_path = "/home/darshan/lunar"
        
    def create_ultra_realistic_heightmap(self):
        """Create ultra-detailed lunar heightmap"""
        
        print("🌙 Creating ultra-realistic lunar heightmap...")
        
        # Load your original lunar image
        original_img = cv2.imread(f"{self.lunar_path}/lunar_surface.jpg", cv2.IMREAD_GRAYSCALE)
        if original_img is None:
            print("❌ Original lunar image not found")
            return False
        
        # Resize to high resolution for detail
        heightmap_size = 1025  # 2^10 + 1 (Gazebo optimal)
        heightmap = cv2.resize(original_img, (heightmap_size, heightmap_size))
        
        # Enhance terrain features
        heightmap = cv2.GaussianBlur(heightmap, (3, 3), 0)
        
        # Add fine surface detail using noise
        detail = np.random.normal(0, 5, (heightmap_size, heightmap_size))
        heightmap = np.clip(heightmap.astype(np.float32) + detail, 0, 255).astype(np.uint8)
        
        # Apply lunar surface characteristics
        heightmap = cv2.bilateralFilter(heightmap, 9, 75, 75)
        
        # Save ultra-resolution heightmap
        output_path = f"{self.lunar_path}/src/lunabot_simulation/media/materials/heightmaps/ultra_lunar_heightmap.png"
        cv2.imwrite(output_path, heightmap)
        
        print(f"✅ Ultra heightmap saved: {output_path}")
        return output_path
        
    def create_realistic_lunar_texture(self):
        """Create photorealistic lunar surface texture"""
        
        print("🎨 Creating photorealistic lunar texture...")
        
        # Load original image
        original = cv2.imread(f"{self.lunar_path}/lunar_surface.jpg")
        if original is None:
            print("❌ Original image not found")
            return False
        
        # Create high-resolution texture
        texture_size = 2048
        texture = cv2.resize(original, (texture_size, texture_size))
        
        # Enhance lunar characteristics
        # Convert to LAB color space for better control
        lab = cv2.cvtColor(texture, cv2.COLOR_BGR2LAB)
        
        # Enhance L channel (luminance) for better contrast
        lab[:,:,0] = cv2.equalizeHist(lab[:,:,0])
        
        # Reduce color saturation for lunar appearance
        lab[:,:,1] = lab[:,:,1] * 0.8  # Reduce A channel
        lab[:,:,2] = lab[:,:,2] * 0.8  # Reduce B channel
        
        # Convert back to BGR
        enhanced_texture = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
        
        # Apply lunar surface effects
        # Add slight blue tint for space lighting
        enhanced_texture[:,:,0] = np.clip(enhanced_texture[:,:,0] * 1.1, 0, 255)  # Blue
        enhanced_texture[:,:,1] = np.clip(enhanced_texture[:,:,1] * 0.95, 0, 255) # Green
        enhanced_texture[:,:,2] = np.clip(enhanced_texture[:,:,2] * 0.9, 0, 255)  # Red
        
        # Save enhanced texture
        texture_path = f"{self.lunar_path}/src/lunabot_simulation/media/materials/textures/ultra_lunar_surface.jpg"
        cv2.imwrite(texture_path, enhanced_texture)
        
        print(f"✅ Ultra texture saved: {texture_path}")
        return texture_path
        
    def create_normal_map(self, heightmap_path):
        """Create normal map for surface detail"""
        
        print("🗺️ Creating surface normal map...")
        
        # Load heightmap
        heightmap = cv2.imread(heightmap_path, cv2.IMREAD_GRAYSCALE)
        if heightmap is None:
            return None
        
        # Calculate gradients
        grad_x = cv2.Sobel(heightmap, cv2.CV_32F, 1, 0, ksize=3)
        grad_y = cv2.Sobel(heightmap, cv2.CV_32F, 0, 1, ksize=3)
        
        # Create normal vectors
        normal_map = np.zeros((heightmap.shape[0], heightmap.shape[1], 3), dtype=np.uint8)
        
        # Normalize gradients and convert to normal map
        strength = 2.0  # Normal map strength
        normal_map[:,:,0] = np.clip((grad_x / 255.0 * strength + 1.0) * 127.5, 0, 255)  # R (X)
        normal_map[:,:,1] = np.clip((grad_y / 255.0 * strength + 1.0) * 127.5, 0, 255)  # G (Y)
        normal_map[:,:,2] = 255  # B (Z) - pointing up
        
        # Save normal map
        normal_path = f"{self.lunar_path}/src/lunabot_simulation/media/materials/textures/lunar_normal.jpg"
        cv2.imwrite(normal_path, normal_map)
        
        print(f"✅ Normal map saved: {normal_path}")
        return normal_path
        
    def create_enhanced_world(self, heightmap_path, texture_path, normal_path):
        """Create world with enhanced lunar surface"""
        
        print("🌍 Creating enhanced lunar world...")
        
        enhanced_world = f'''<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="ultra_realistic_lunar">
    
    <!-- Ultra-realistic Lunar Physics -->
    <physics name="lunar_physics" default="1" type="ode">
      <gravity>0 0 -1.62</gravity>
      <ode>
        <solver>
          <type>world</type>
          <iters>200</iters>
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>1e-7</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000</contact_max_correcting_vel>
          <contact_surface_layer>0.0001</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.0001</max_step_size>
    </physics>

    <!-- Professional Lunar Lighting -->
    <light name="harsh_sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>400 200 500 0 0 0</pose>
      <diffuse>4.0 4.0 3.8 1</diffuse>
      <specular>3.5 3.5 3.3 1</specular>
      <attenuation>
        <range>3000</range>
        <constant>0.7</constant>
        <linear>0.003</linear>
        <quadratic>0.0003</quadratic>
      </attenuation>
      <direction>-0.6 -0.4 -0.8</direction>
    </light>
    
    <!-- Earth-shine (subtle blue fill) -->
    <light name="earth_reflection" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>-300 -150 300 0 0 0</pose>
      <diffuse>0.2 0.3 0.4 1</diffuse>
      <direction>0.4 0.3 -0.5</direction>
    </light>
    
    <!-- Ambient space light -->
    <light name="star_light" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>0 0 200 0 0 0</pose>
      <diffuse>0.1 0.1 0.1 1</diffuse>
      <direction>0 0 -1</direction>
    </light>

    <!-- Space Environment -->
    <scene>
      <ambient>0.02 0.02 0.02 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
      <fog>
        <color>0.0 0.0 0.0 1</color>
        <type>none</type>
      </fog>
    </scene>

    <!-- Ultra-Realistic Lunar Surface -->
    <model name="ultra_lunar_surface">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="surface_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_path}</uri>
              <size>600 600 30</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.7</mu>
                <mu2>0.7</mu2>
                <slip1>0.02</slip1>
                <slip2>0.02</slip2>
                <fdir1>0 0 0</fdir1>
              </ode>
            </friction>
            <contact>
              <ode>
                <soft_cfm>1e-8</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+13</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
            </contact>
            <bounce>
              <restitution_coefficient>0.05</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <geometry>
            <heightmap>
              <uri>{heightmap_path}</uri>
              <size>600 600 30</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>{texture_path}</diffuse>
                <normal>{normal_path}</normal>
                <size>40</size>
              </texture>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
            <specular>0.15 0.15 0.15 1</specular>
            <emissive>0.0 0.0 0.0 1</emissive>
            <pbr>
              <metal>
                <metalness_map>{texture_path}</metalness_map>
                <roughness_map>{normal_path}</roughness_map>
              </metal>
            </pbr>
          </material>
        </visual>
      </link>
    </model>

    <!-- Enhanced Crater Details -->
    <model name="detailed_crater_1">
      <static>true</static>
      <pose>50 30 0 0 0 0</pose>
      <link name="crater_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://lunar_crater_detailed</uri>
              <scale>12 12 6</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://lunar_crater_detailed</uri>
              <scale>12 12 6</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.05 0.05 0.05 1</ambient>
            <diffuse>0.2 0.2 0.2 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Realistic Boulder Field -->
    <model name="boulder_field_detailed">
      <static>true</static>
      <pose>-40 -30 3 0.2 0.1 0.8</pose>
      <link name="boulder_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>model://lunar_boulder_cluster</uri>
              <scale>8 6 8</scale>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>model://lunar_boulder_cluster</uri>
              <scale>8 6 8</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ROS Integration -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>30.0</update_rate>
    </plugin>

    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so" />

    <!-- Enhanced Camera View -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>40 -40 25 0 0.4 0.785</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
        <track_visual>
          <name>custom_rover</name>
          <min_dist>8</min_dist>
          <max_dist>150</max_dist>
        </track_visual>
      </camera>
    </gui>

  </world>
</sdf>'''

        # Save enhanced world
        world_path = f"{self.lunar_path}/src/lunabot_simulation/worlds/ultra_realistic_lunar.world"
        with open(world_path, 'w') as f:
            f.write(enhanced_world)
        
        print(f"✅ Enhanced world saved: {world_path}")
        return world_path
        
    def run_enhancement(self):
        """Run complete surface enhancement"""
        
        print("🚀 LUNAR SURFACE ENHANCEMENT STARTED")
        print("=" * 50)
        
        # Create ultra-realistic assets
        heightmap_path = self.create_ultra_realistic_heightmap()
        texture_path = self.create_realistic_lunar_texture()
        normal_path = self.create_normal_map(heightmap_path)
        world_path = self.create_enhanced_world(heightmap_path, texture_path, normal_path)
        
        print("=" * 50)
        print("🎉 LUNAR SURFACE ENHANCEMENT COMPLETE!")
        print("=" * 50)
        print(f"🌙 Enhanced heightmap: {heightmap_path}")
        print(f"🎨 Ultra texture: {texture_path}")
        print(f"🗺️ Normal map: {normal_path}")
        print(f"🌍 Enhanced world: {world_path}")
        print("=" * 50)

def main():
    enhancer = AdvancedLunarSurfaceEnhancer()
    enhancer.run_enhancement()

if __name__ == "__main__":
    main()
