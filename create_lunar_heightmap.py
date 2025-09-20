#!/usr/bin/env python3
import numpy as np
from PIL import Image
import noise
import random

def generate_lunar_terrain(width=512, height=512):
    """Generate realistic lunar terrain heightmap"""
    
    # Create base terrain using Perlin noise
    terrain = np.zeros((height, width))
    
    # Multiple octaves for realistic terrain
    for y in range(height):
        for x in range(width):
            # Base terrain
            terrain[y,x] = noise.pnoise2(x/100.0, y/100.0, octaves=4, persistence=0.5, lacunarity=2.0)
            
            # Add large scale features
            terrain[y,x] += 0.3 * noise.pnoise2(x/200.0, y/200.0, octaves=2)
            
            # Add fine details
            terrain[y,x] += 0.1 * noise.pnoise2(x/50.0, y/50.0, octaves=6)
    
    # Add craters
    num_craters = random.randint(15, 25)
    for _ in range(num_craters):
        cx = random.randint(50, width-50)
        cy = random.randint(50, height-50)
        radius = random.randint(20, 80)
        depth = random.uniform(0.1, 0.4)
        
        for y in range(max(0, cy-radius), min(height, cy+radius)):
            for x in range(max(0, cx-radius), min(width, cx+radius)):
                dist = np.sqrt((x-cx)**2 + (y-cy)**2)
                if dist < radius:
                    # Create crater depression
                    crater_factor = 1.0 - (dist/radius)
                    terrain[y,x] -= depth * crater_factor * crater_factor
    
    # Normalize to 0-255 range
    terrain = (terrain - terrain.min()) / (terrain.max() - terrain.min())
    terrain = (terrain * 255).astype(np.uint8)
    
    return terrain

# Generate heightmap
print("Generating lunar terrain heightmap...")
heightmap = generate_lunar_terrain(512, 512)
heightmap_img = Image.fromarray(heightmap, 'L')
heightmap_img.save('/home/darshan/lunar/src/lunabot_simulation/media/materials/heightmaps/lunar_heightmap.png')
print("Lunar heightmap saved!")

# Generate realistic lunar surface texture
def generate_lunar_texture(width=1024, height=1024):
    """Generate realistic lunar surface texture"""
    texture = np.zeros((height, width, 3), dtype=np.uint8)
    
    # Base gray color similar to lunar regolith
    base_color = [120, 115, 110]  # Grayish-brown like real Moon
    
    for y in range(height):
        for x in range(width):
            # Add noise for surface texture
            noise_val = noise.pnoise2(x/20.0, y/20.0, octaves=4, persistence=0.6)
            
            # Vary brightness based on noise
            brightness = 0.8 + 0.4 * noise_val
            
            # Apply to each channel
            for c in range(3):
                texture[y,x,c] = int(base_color[c] * brightness)
                texture[y,x,c] = np.clip(texture[y,x,c], 0, 255)
    
    return texture

print("Generating lunar surface texture...")
texture = generate_lunar_texture(1024, 1024)
texture_img = Image.fromarray(texture, 'RGB')
texture_img.save('/home/darshan/lunar/src/lunabot_simulation/media/materials/textures/lunar_surface.jpg')
print("Lunar texture saved!")

print("✅ Lunar terrain generation complete!")
