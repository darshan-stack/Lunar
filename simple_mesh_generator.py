#!/usr/bin/env python3
import numpy as np
import cv2
import os

def create_obj_terrain(image_path, output_path, terrain_size=100, height_scale=8):
    """Create OBJ terrain file from image (no Blender required)"""
    
    print(f"🌙 Creating terrain mesh from: {image_path}")
    
    # Load and process image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError(f"Could not load: {image_path}")
    
    # Resize to manageable size
    img = cv2.resize(img, (256, 256))
    heights = img.astype(np.float32) / 255.0 * height_scale
    
    print(f"📏 Terrain size: {img.shape}, Height range: 0-{height_scale}m")
    
    # Create vertices
    vertices = []
    faces = []
    
    rows, cols = heights.shape
    
    # Generate vertices
    for y in range(rows):
        for x in range(cols):
            world_x = (x / cols) * terrain_size - terrain_size/2
            world_y = (y / rows) * terrain_size - terrain_size/2  
            world_z = heights[y, x]
            vertices.append([world_x, world_y, world_z])
    
    # Generate faces (triangles)
    for y in range(rows - 1):
        for x in range(cols - 1):
            # Current vertex indices
            v1 = y * cols + x
            v2 = y * cols + (x + 1)
            v3 = (y + 1) * cols + x
            v4 = (y + 1) * cols + (x + 1)
            
            # Two triangles per quad (OBJ uses 1-based indexing)
            faces.append([v1+1, v2+1, v3+1])
            faces.append([v2+1, v4+1, v3+1])
    
    # Create output directory
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    # Write OBJ file
    with open(output_path, 'w') as f:
        f.write("# Lunar terrain mesh generated from image\n")
        f.write(f"# Vertices: {len(vertices)}, Faces: {len(faces)}\n")
        
        # Write vertices
        for vertex in vertices:
            f.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
        
        # Write faces
        for face in faces:
            f.write(f"f {face[0]} {face[1]} {face[2]}\n")
    
    print(f"✅ Terrain mesh saved: {output_path}")
    print(f"📊 Statistics: {len(vertices)} vertices, {len(faces)} faces")
    
    return output_path

def create_crater_objects(output_dir, num_craters=10):
    """Create separate crater objects"""
    
    crater_files = []
    
    for i in range(num_craters):
        crater_path = os.path.join(output_dir, f"crater_{i:02d}.obj")
        
        # Random crater parameters
        radius = np.random.uniform(2, 6)
        depth = radius * 0.3
        
        # Simple crater geometry (inverted hemisphere)
        vertices = []
        faces = []
        
        # Center vertex (bottom of crater)
        vertices.append([0, 0, -depth])
        
        # Rim vertices
        rim_points = 12
        for j in range(rim_points):
            angle = j * 2 * np.pi / rim_points
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            vertices.append([x, y, 0])
        
        # Create faces (fan triangulation)
        for j in range(rim_points):
            next_j = (j + 1) % rim_points
            faces.append([1, j+2, next_j+2])  # OBJ 1-based indexing
        
        # Write crater OBJ
        with open(crater_path, 'w') as f:
            f.write(f"# Crater {i}\n")
            for vertex in vertices:
                f.write(f"v {vertex[0]:.6f} {vertex[1]:.6f} {vertex[2]:.6f}\n")
            for face in faces:
                f.write(f"f {face[0]} {face[1]} {face[2]}\n")
        
        crater_files.append(crater_path)
    
    print(f"✅ Created {len(crater_files)} crater objects")
    return crater_files

def create_enhanced_world_file(world_path, terrain_mesh, crater_meshes):
    """Create Gazebo world with terrain and crater objects"""
    
    world_content = f'''<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="enhanced_lunar">
    
    <!-- Lunar Physics -->
    <physics name="default_physics" default="0" type="ode">
      <gravity>0 0 -1.62</gravity>
    </physics>

    <!-- Enhanced Lighting -->
    <light name="sun" type="directional">
      <cast_shadows>1</cast_shadows>
      <pose>80 40 150 0 0 0</pose>
      <diffuse>1.3 1.3 1.2 1</diffuse>
      <specular>1.0 1.0 0.95 1</specular>
      <direction>-0.5 -0.3 -0.8</direction>
    </light>
    
    <light name="fill_light" type="directional">
      <cast_shadows>0</cast_shadows>
      <pose>-40 -20 80 0 0 0</pose>
      <diffuse>0.2 0.2 0.2 1</diffuse>
      <direction>0.3 0.2 -0.5</direction>
    </light>

    <!-- Space Environment -->
    <scene>
      <ambient>0.12 0.12 0.12 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- Main Lunar Terrain -->
    <model name="lunar_terrain_mesh">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://meshes/lunar_terrain.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.4</mu>
                <mu2>0.4</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        
        <visual name="visual">
          <cast_shadows>1</cast_shadows>
          <geometry>
            <mesh>
              <uri>file://meshes/lunar_terrain.obj</uri>
              <scale>1 1 1</scale>
            </mesh>
          </geometry>
          <material>
            <ambient>0.25 0.25 0.25 1</ambient>
            <diffuse>0.4 0.4 0.4 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>
'''

    # Add crater objects
    for i, crater_mesh in enumerate(crater_meshes):
        # Random positions for craters
        x = np.random.uniform(-40, 40)
        y = np.random.uniform(-40, 40)
        
        world_content += f'''
    <!-- Crater {i+1} -->
    <model name="crater_{i+1:02d}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 {np.random.uniform(0, 2*np.pi)}</pose>
      <link name="crater_link">
        <collision name="collision">
          <geometry>
            <mesh>
              <uri>file://meshes/crater_{i:02d}.obj</uri>
            </mesh>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <mesh>
              <uri>file://meshes/crater_{i:02d}.obj</uri>
            </mesh>
          </geometry>
          <material>
            <ambient>0.15 0.15 0.15 1</ambient>
            <diffuse>0.25 0.25 0.25 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>'''

    # Add ROS plugins
    world_content += '''
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

    # Write world file
    os.makedirs(os.path.dirname(world_path), exist_ok=True)
    with open(world_path, 'w') as f:
        f.write(world_content)
    
    print(f"✅ Enhanced world created: {world_path}")

def main():
    """Generate complete mesh-based lunar environment"""
    
    print("🚀 GENERATING ENHANCED LUNAR ENVIRONMENT")
    print("=" * 50)
    
    # Paths
    image_path = "/home/darshan/lunar/lunar_surface.jpg"
    mesh_dir = "/home/darshan/lunar/src/lunabot_simulation/meshes"
    terrain_mesh = os.path.join(mesh_dir, "lunar_terrain.obj")
    world_path = "/home/darshan/lunar/src/lunabot_simulation/worlds/enhanced_lunar.world"
    
    try:
        # Create main terrain mesh
        create_obj_terrain(image_path, terrain_mesh)
        
        # Create crater objects  
        crater_meshes = create_crater_objects(mesh_dir, num_craters=8)
        
        # Create enhanced world file
        create_enhanced_world_file(world_path, terrain_mesh, crater_meshes)
        
        print("=" * 50)
        print("🎉 ENHANCED LUNAR ENVIRONMENT COMPLETE!")
        print("=" * 50)
        print(f"🌍 World file: {world_path}")
        print(f"🗻 Main terrain: {terrain_mesh}")
        print(f"🕳️  Craters: {len(crater_meshes)} objects")
        print("=" * 50)
        print("🚀 Ready to launch enhanced simulation!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())
