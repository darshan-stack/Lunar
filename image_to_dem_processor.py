#!/usr/bin/env python3
import cv2
import numpy as np
import os
from PIL import Image
import json

class LunarTerrainProcessor:
    """Convert lunar surface images to DEM and Gazebo world"""
    
    def __init__(self, image_path):
        self.image_path = image_path
        print(f"📸 Loading image: {image_path}")
        
        # Load and process image
        self.original_image = cv2.imread(image_path)
        if self.original_image is None:
            raise FileNotFoundError(f"Could not load image: {image_path}")
            
        self.gray_image = cv2.cvtColor(self.original_image, cv2.COLOR_BGR2GRAY)
        self.elevation_map = None
        self.craters = []
        self.rocks = []
        
        print(f"✅ Image loaded: {self.gray_image.shape}")
        
    def analyze_terrain(self):
        """Extract terrain features from image"""
        
        print("🔍 Analyzing terrain features...")
        
        # Create elevation map from image intensity
        self.elevation_map = self.create_elevation_from_image()
        
        # Detect craters
        self.craters = self.detect_craters()
        
        # Detect rocks
        self.rocks = self.detect_rocks()
        
        print(f"✅ Found {len(self.craters)} craters, {len(self.rocks)} rocks")
        
    def create_elevation_from_image(self):
        """Convert image brightness to elevation data"""
        
        # Apply Gaussian blur to smooth terrain
        blurred = cv2.GaussianBlur(self.gray_image, (5, 5), 0)
        
        # Convert to elevation (0-50m range for realistic lunar terrain)
        elevation = blurred.astype(np.float32) / 255.0 * 50.0
        
        return elevation
        
    def detect_craters(self):
        """Detect circular craters using Hough circles"""
        
        # Apply edge detection
        edges = cv2.Canny(self.gray_image, 50, 150)
        
        # Detect circles
        circles = cv2.HoughCircles(
            edges,
            cv2.HOUGH_GRADIENT,
            dp=1,
            minDist=30,
            param1=50,
            param2=30,
            minRadius=10,
            maxRadius=100
        )
        
        craters = []
        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                craters.append({
                    'x': int(x),
                    'y': int(y),
                    'radius': int(r),
                    'depth': float(r * 0.1)  # Depth proportional to radius
                })
        
        return craters
        
    def detect_rocks(self):
        """Detect rocks using adaptive thresholding"""
        
        # Apply adaptive threshold
        adaptive_thresh = cv2.adaptiveThreshold(
            self.gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
            cv2.THRESH_BINARY, 11, 2
        )
        
        # Find contours
        contours, _ = cv2.findContours(
            adaptive_thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        
        rocks = []
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by size
            if 50 < area < 2000:
                x, y, w, h = cv2.boundingRect(contour)
                rocks.append({
                    'x': int(x + w//2),
                    'y': int(y + h//2),
                    'width': int(w),
                    'height': int(h),
                    'elevation': float(w * h * 0.001)  # Height based on area
                })
        
        return rocks
        
    def save_dem_heightmap(self, output_path):
        """Save DEM as heightmap for Gazebo"""
        
        print(f"💾 Saving heightmap: {output_path}")
        
        # Create directory if needed
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # Convert to 8-bit grayscale for Gazebo
        heightmap_8bit = (self.elevation_map / 50.0 * 255).astype(np.uint8)
        
        # Save heightmap
        cv2.imwrite(output_path, heightmap_8bit)
        
        print(f"✅ Heightmap saved: {output_path}")
        
    def create_lunar_texture(self, output_path):
        """Create realistic lunar surface texture"""
        
        print(f"🎨 Creating texture: {output_path}")
        
        # Create directory if needed
        os.makedirs(os.path.dirname(output_path), exist_ok=True)
        
        # Resize original image for texture
        texture = cv2.resize(self.original_image, (1024, 1024))
        
        # Enhance lunar characteristics (grayish, low contrast)
        gray_texture = cv2.cvtColor(texture, cv2.COLOR_BGR2GRAY)
        
        # Convert back to 3-channel with lunar tint
        lunar_texture = np.zeros((1024, 1024, 3), dtype=np.uint8)
        lunar_texture[:,:,0] = gray_texture  # Blue channel
        lunar_texture[:,:,1] = gray_texture  # Green channel  
        lunar_texture[:,:,2] = np.clip(gray_texture * 1.1, 0, 255)  # Red channel (slightly enhanced)
        
        # Save texture
        cv2.imwrite(output_path, lunar_texture)
        
        print(f"✅ Texture saved: {output_path}")
        
    def create_gazebo_world(self, world_path, heightmap_path, texture_path):
        """Create Gazebo world file with realistic lunar environment"""
        
        print(f"🌍 Creating Gazebo world: {world_path}")
        
        # Create directory if needed
        os.makedirs(os.path.dirname(world_path), exist_ok=True)
        
        # World file content
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

    <!-- Space background (pure black) -->
    <scene>
      <ambient>0.05 0.05 0.05 1</ambient>
      <background>0.0 0.0 0.0 1</background>
      <shadows>1</shadows>
    </scene>

    <!-- Lunar Terrain from Your Image -->
    <model name="lunar_terrain">
      <static>true</static>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>file://media/materials/heightmaps/lunar_heightmap.png</uri>
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
              <uri>file://media/materials/heightmaps/lunar_heightmap.png</uri>
              <size>400 400 50</size>
              <pos>0 0 0</pos>
              <texture>
                <diffuse>file://media/materials/textures/lunar_surface.jpg</diffuse>
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

    <!-- ROS Integration Plugins -->
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
        with open(world_path, 'w') as f:
            f.write(world_content)
        
        print(f"✅ Gazebo world created: {world_path}")
        
    def save_analysis_report(self, output_path):
        """Save terrain analysis report"""
        
        report = {
            'source_image': self.image_path,
            'terrain_size': f"{self.gray_image.shape[1]}x{self.gray_image.shape[0]} pixels",
            'elevation_range': f"{np.min(self.elevation_map):.1f} - {np.max(self.elevation_map):.1f} meters",
            'detected_features': {
                'craters': len(self.craters),
                'rocks': len(self.rocks)
            },
            'crater_details': self.craters[:5],  # First 5 craters
            'rock_details': self.rocks[:10]     # First 10 rocks
        }
        
        with open(output_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        print(f"📊 Analysis report saved: {output_path}")

def main():
    """Main processing pipeline"""
    
    print("🚀 Starting Lunar Terrain Processing Pipeline...")
    print("=" * 50)
    
    # Input and output paths
    image_path = "/home/darshan/lunar/lunar_surface.jpg"
    heightmap_path = "/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap.png"
    texture_path = "/home/darshan/lunar/src/lunabot_simulation/media/materials/textures/lunar_surface.jpg"
    world_path = "/home/darshan/lunar/src/lunabot_simulation/worlds/lunar_photorealistic.world"
    report_path = "/home/darshan/lunar/processed_terrain/analysis_report.json"
    
    try:
        # Initialize processor
        processor = LunarTerrainProcessor(image_path)
        
        # Analyze terrain
        processor.analyze_terrain()
        
        # Generate all outputs
        processor.save_dem_heightmap(heightmap_path)
        processor.create_lunar_texture(texture_path)
        processor.create_gazebo_world(world_path, heightmap_path, texture_path)
        processor.save_analysis_report(report_path)
        
        print("=" * 50)
        print("🎉 PIPELINE COMPLETED SUCCESSFULLY!")
        print("=" * 50)
        print(f"📁 Heightmap: {heightmap_path}")
        print(f"🎨 Texture: {texture_path}")
        print(f"🌍 World file: {world_path}")
        print(f"📊 Report: {report_path}")
        print("=" * 50)
        print("🚀 Ready to launch in Gazebo!")
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
        
    return 0

if __name__ == "__main__":
    exit(main())
