#!/usr/bin/env python3

def create_improved_world():
    """Create visually improved lunar world"""
    
    world_content = '''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="lunar_photorealistic">
    
    <!-- Lunar Physics -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -1.62</gravity>
    </physics>

    <!-- IMPROVED LIGHTING -->
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>50 30 100 0 0 0</pose>
      <diffuse>1.2 1.2 1.1 1</diffuse>
      <specular>1.0 1.0 0.95 1</specular>
      <direction>-0.5 -0.3 -0.8</direction>
    </light>
    
    <!-- Additional ambient light for visibility -->
    <light name="ambient_fill" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>-50 -30 80 0 0 0</pose>
      <diffuse>0.3 0.3 0.3 1</diffuse>
      <direction>0.5 0.3 -0.5</direction>
    </light>

    <!-- IMPROVED SCENE -->
    <scene>
      <ambient>0.15 0.15 0.15 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
      <sky>
        <clouds>
          <speed>12</speed>
        </clouds>
      </sky>
    </scene>

    <!-- SCALED DOWN TERRAIN -->
    <model name="lunar_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap_fixed.png</uri>
              <size>400 400 15</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.3</mu>
                <mu2>0.3</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <geometry>
            <heightmap>
              <uri>/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap_fixed.png</uri>
              <size>400 400 15</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>/home/darshan/lunar/src/lunabot_simulation/media/materials/textures/lunar_surface.jpg</diffuse>
                <size>20</size>
              </texture>
            </heightmap>
          </geometry>
          <material>
            <ambient>0.4 0.4 0.4 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
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
    
    # Write improved world file
    with open('/home/darshan/lunar/src/lunabot_simulation/worlds/lunar_improved.world', 'w') as f:
        f.write(world_content)
    
    print("✅ Improved world file created!")

if __name__ == "__main__":
    create_improved_world()
