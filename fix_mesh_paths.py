#!/usr/bin/env python3
import os

def create_working_world():
    """Create world with working absolute paths"""
    
    mesh_dir = "/home/darshan/lunar/src/lunabot_simulation/meshes"
    
    world_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="working_lunar">
    
    <!-- Lunar Physics -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -1.62</gravity>
    </physics>

    <!-- PERFECT LIGHTING -->
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>100 50 200 0 0 0</pose>
      <diffuse>1.5 1.5 1.4 1</diffuse>
      <specular>1.2 1.2 1.1 1</specular>
      <direction>-0.5 -0.3 -0.8</direction>
    </light>
    
    <light name="fill_light" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>-50 -25 100 0 0 0</pose>
      <diffuse>0.4 0.4 0.4 1</diffuse>
      <direction>0.3 0.2 -0.5</direction>
    </light>

    <!-- Space Environment -->
    <scene>
      <ambient>0.2 0.2 0.2 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- WORKING Terrain Mesh -->
    <model name="lunar_terrain_working">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>{mesh_dir}/lunar_terrain.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.6</mu>
                <mu2>0.6</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <geometry>
            <mesh>
              <uri>{mesh_dir}/lunar_terrain.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.5 0.5 0.5 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- Simple Rock Obstacles (instead of complex craters) -->
    <model name="rock_1">
      <static>true</static>
      <pose>15 10 2 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>3 2 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>3 2 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="rock_2">
      <static>true</static>
      <pose>-20 15 1.5 0 0 0.5</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>2</radius>
              <length>3</length>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>2</radius>
              <length>3</length>
            </cylinder>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name="rock_3">
      <static>true</static>
      <pose>-10 -20 1 0 0 1.2</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>2 3 2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>2 3 2</size>
            </box>
          </geometry>
          <material>
            <ambient>0.2 0.2 0.2 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- ROS Integration -->
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>1.0</update_rate>
    </plugin>

    <plugin name="gazebo_ros_factory" filename="libgazebo_ros_factory.so" />

  </world>
</sdf>'''

    # Write working world
    world_path = "/home/darshan/lunar/src/lunabot_simulation/worlds/working_lunar.world"
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    
    with open(world_path, 'w') as f:
        f.write(world_content)
    
    print(f"✅ Working world created: {world_path}")

if __name__ == "__main__":
    create_working_world()
