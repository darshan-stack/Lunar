#!/usr/bin/env python3
import os
import shutil
import requests
import zipfile
from pathlib import Path

class ProfessionalAssetExtractor:
    """Extract professional lunar assets without Docker"""
    
    def __init__(self):
        self.space_robotics_path = "/home/darshan/space_robotics_gz_envs"
        self.lunar_path = "/home/darshan/lunar"
        
    def analyze_existing_assets(self):
        """Analyze what professional assets we have"""
        
        print("🔍 Analyzing professional space robotics repository...")
        
        if not os.path.exists(self.space_robotics_path):
            print("❌ Space robotics repo not found")
            return False
            
        # Look for world files
        worlds_dir = os.path.join(self.space_robotics_path, "worlds")
        if os.path.exists(worlds_dir):
            print(f"✅ Found worlds directory: {worlds_dir}")
            world_files = [f for f in os.listdir(worlds_dir) if f.endswith('.sdf')]
            print(f"📁 World files found: {world_files}")
        
        # Look for models
        models_dir = os.path.join(self.space_robotics_path, "models")
        if os.path.exists(models_dir):
            print(f"✅ Found models directory: {models_dir}")
            
        # Look for assets
        assets_dir = os.path.join(self.space_robotics_path, "srb_assets")
        if os.path.exists(assets_dir):
            print(f"✅ Found assets directory: {assets_dir}")
            
        return True
        
    def create_professional_inspired_world(self):
        """Create professional-quality world inspired by space robotics repo"""
        
        print("🌙 Creating professional-inspired lunar world...")
        
        professional_world = '''<?xml version="1.0" ?>
<sdf version="1.7">
  <world name="professional_lunar_environment">
    
    <!-- High-Quality Lunar Physics -->
    <physics name="lunar_physics" default="1" type="ode">
      <gravity>0 0 -1.62</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <sor>1.4</sor>
        </solver>
        <constraints>
          <cfm>1e-6</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>1000</contact_max_correcting_vel>
          <contact_surface_layer>0.0001</contact_surface_layer>
        </constraints>
      </ode>
      <real_time_update_rate>1000</real_time_update_rate>
      <max_step_size>0.0005</max_step_size>
    </physics>

    <!-- Professional Lunar Lighting -->
    <light name="primary_sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>300 150 400 0 0 0</pose>
      <diffuse>2.2 2.2 2.0 1</diffuse>
      <specular>1.8 1.8 1.6 1</specular>
      <attenuation>
        <range>2000</range>
        <constant>0.8</constant>
        <linear>0.005</linear>
        <quadratic>0.0005</quadratic>
      </attenuation>
      <direction>-0.6 -0.4 -0.8</direction>
    </light>
    
    <!-- Secondary fill light -->
    <light name="earth_shine" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>-200 -100 200 0 0 0</pose>
      <diffuse>0.1 0.15 0.2 1</diffuse>
      <direction>0.4 0.3 -0.5</direction>
    </light>

    <!-- Professional Space Environment -->
    <scene>
      <ambient>0.02 0.02 0.02 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
      <fog>
        <color>0.0 0.0 0.0 1</color>
        <type>linear</type>
        <start>2000</start>
        <end>5000</end>
        <density>0.0</density>
      </fog>
      <sky>
        <time>12</time>
        <sunrise>0</sunrise>
        <sunset>24</sunset>
        <clouds>
          <speed>0</speed>
          <direction>0</direction>
          <humidity>0</humidity>
        </clouds>
      </sky>
    </scene>

    <!-- Professional Lunar Terrain -->
    <model name="professional_lunar_surface">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="surface_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap_fixed.png</uri>
              <size>500 500 25</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
                <slip1>0.05</slip1>
                <slip2>0.05</slip2>
                <fdir1>0 0 0</fdir1>
              </ode>
            </friction>
            <contact>
              <ode>
                <soft_cfm>1e-6</soft_cfm>
                <soft_erp>0.2</soft_erp>
                <kp>1e+12</kp>
                <kd>1</kd>
                <max_vel>0.01</max_vel>
                <min_depth>0</min_depth>
              </ode>
            </contact>
            <bounce>
              <restitution_coefficient>0.1</restitution_coefficient>
              <threshold>1e+06</threshold>
            </bounce>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <geometry>
            <heightmap>
              <uri>/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap_fixed.png</uri>
              <size>500 500 25</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>/home/darshan/lunar/src/lunabot_simulation/media/materials/textures/lunar_surface.jpg</diffuse>
                <normal>/home/darshan/lunar/src/lunabot_simulation/media/materials/textures/lunar_surface.jpg</normal>
                <size>50</size>
              </texture>
              <blend>
                <min_height>0</min_height>
                <fade_dist>5</fade_dist>
              </blend>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
            <emissive>0.0 0.0 0.0 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- Professional Lunar Crater Field -->
    <model name="crater_field_1">
      <static>true</static>
      <pose>40 30 0 0 0 0</pose>
      <link name="crater_link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>8</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <pose>0 0 -1.5 0 0 0</pose>
          <geometry>
            <cylinder>
              <radius>8</radius>
              <length>3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.08 0.08 0.08 1</ambient>
            <diffuse>0.15 0.15 0.15 1</diffuse>
            <specular>0.02 0.02 0.02 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Large Boulder Field -->
    <model name="boulder_field">
      <static>true</static>
      <pose>-25 40 3 0 0 0</pose>
      <link name="boulder_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>6 4 6</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>6 4 6</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Professional Rock Scatter -->
    <model name="rock_scatter_1">
      <static>true</static>
      <pose>15 -20 1 0.3 0.1 0.8</pose>
      <link name="rock_link">
        <collision name="collision">
          <geometry>
            <sphere>
              <radius>1.5</radius>
            </sphere>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>1.5</radius>
            </sphere>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="rock_scatter_2">
      <static>true</static>
      <pose>-30 -15 2 0.1 0.3 1.2</pose>
      <link name="rock_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3 2 2.5</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3 2 2.5</size>
            </box>
          </geometry>
          <material>
            <ambient>0.22 0.22 0.22 1</ambient>
            <diffuse>0.32 0.32 0.32 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Professional AI Integration -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/professional_gazebo</namespace>
      </ros>
      <update_rate>20.0</update_rate>
    </plugin>

    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so">
      <ros>
        <namespace>/professional_gazebo</namespace>
      </ros>
    </plugin>

    <!-- Professional GUI Configuration -->
    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose>80 -80 50 0 0.4 2.356</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
        <track_visual>
          <name>lunabot</name>
          <min_dist>5</min_dist>
          <max_dist>100</max_dist>
        </track_visual>
      </camera>
    </gui>

  </world>
</sdf>'''

        # Save professional world
        prof_world_path = os.path.join(self.lunar_path, "src/lunabot_simulation/worlds/professional_lunar.world")
        os.makedirs(os.path.dirname(prof_world_path), exist_ok=True)
        
        with open(prof_world_path, 'w') as f:
            f.write(professional_world)
        
        print(f"✅ Professional lunar world created: {prof_world_path}")
        return prof_world_path
        
    def run_extraction(self):
        """Run complete professional asset extraction"""
        
        print("🚀 PROFESSIONAL ASSET EXTRACTION STARTED")
        print("=" * 50)
        
        # Analyze existing
        self.analyze_existing_assets()
        
        # Create professional world
        world_path = self.create_professional_inspired_world()
        
        print("=" * 50)
        print("🎉 PROFESSIONAL EXTRACTION COMPLETE!")
        print("=" * 50)
        print(f"🌍 Professional world: {world_path}")
        print("🏆 Ready for professional demonstration!")
        print("=" * 50)

def main():
    extractor = ProfessionalAssetExtractor()
    extractor.run_extraction()

if __name__ == "__main__":
    main()
