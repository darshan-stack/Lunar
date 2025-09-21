import bpy
import bmesh
from mathutils import Vector
import numpy as np
import cv2
from mathutils.noise import noise, fractal

def clear_scene():
    """Clear default scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

def create_lunar_terrain_from_image(image_path, terrain_size=100, height_scale=10):
    """Create realistic lunar terrain from image"""
    
    print(f"🌙 Creating lunar terrain from: {image_path}")
    
    # Load heightmap image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        print(f"❌ Could not load image: {image_path}")
        return None
    
    # Resize to manageable size for Blender
    img = cv2.resize(img, (512, 512))
    
    # Normalize to height values
    heights = img.astype(np.float32) / 255.0 * height_scale
    
    # Create mesh
    bpy.ops.mesh.primitive_plane_add(size=terrain_size, location=(0, 0, 0))
    terrain = bpy.context.active_object
    terrain.name = "LunarTerrain"
    
    # Enter edit mode and subdivide
    bpy.context.view_layer.objects.active = terrain
    bpy.ops.object.mode_set(mode='EDIT')
    
    # Subdivide to create detailed mesh
    for i in range(9):  # Creates 513x513 vertices
        bpy.ops.mesh.subdivide()
    
    # Get mesh data
    bm = bmesh.from_mesh(terrain.data)
    
    # Apply heightmap data
    for i, vert in enumerate(bm.verts):
        x_idx = int((vert.co.x + terrain_size/2) / terrain_size * 511)
        y_idx = int((vert.co.y + terrain_size/2) / terrain_size * 511)
        
        x_idx = max(0, min(511, x_idx))
        y_idx = max(0, min(511, y_idx))
        
        vert.co.z = heights[y_idx, x_idx]
    
    # Update mesh
    bm.to_mesh(terrain.data)
    bm.free()
    
    bpy.ops.object.mode_set(mode='OBJECT')
    
    # Add lunar material
    create_lunar_material(terrain)
    
    return terrain

def create_lunar_material(obj):
    """Create realistic lunar surface material"""
    
    # Create material
    mat = bpy.data.materials.new(name="LunarSurface")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    links = mat.node_tree.links
    
    # Clear default nodes
    nodes.clear()
    
    # Add nodes
    output = nodes.new(type='ShaderNodeOutputMaterial')
    principled = nodes.new(type='ShaderNodeBsdfPrincipled')
    noise = nodes.new(type='ShaderNodeTexNoise')
    color_ramp = nodes.new(type='ShaderNodeValToRGB')
    mapping = nodes.new(type='ShaderNodeMapping')
    coord = nodes.new(type='ShaderNodeTexCoord')
    
    # Configure lunar surface properties
    principled.inputs['Base Color'].default_value = (0.3, 0.3, 0.3, 1)  # Gray
    principled.inputs['Roughness'].default_value = 0.9  # Very rough
    principled.inputs['Specular'].default_value = 0.1   # Low reflectivity
    
    # Add surface detail with noise
    noise.inputs['Scale'].default_value = 5.0
    noise.inputs['Detail'].default_value = 10.0
    
    # Configure color ramp for surface variation
    color_ramp.color_ramp.elements[0].color = (0.2, 0.2, 0.2, 1)
    color_ramp.color_ramp.elements[1].color = (0.4, 0.4, 0.4, 1)
    
    # Link nodes
    links.new(coord.outputs['Generated'], mapping.inputs['Vector'])
    links.new(mapping.outputs['Vector'], noise.inputs['Vector'])
    links.new(noise.outputs['Fac'], color_ramp.inputs['Fac'])
    links.new(color_ramp.outputs['Color'], principled.inputs['Base Color'])
    links.new(principled.outputs['BSDF'], output.inputs['Surface'])
    
    # Apply material
    obj.data.materials.append(mat)

def create_craters(terrain, num_craters=20):
    """Add realistic craters to terrain"""
    
    for i in range(num_craters):
        # Random crater position and size
        x = np.random.uniform(-40, 40)
        y = np.random.uniform(-40, 40)
        radius = np.random.uniform(2, 8)
        depth = radius * 0.3
        
        # Create crater
        bpy.ops.mesh.primitive_uv_sphere_add(radius=radius, location=(x, y, -depth/2))
        crater = bpy.context.active_object
        crater.name = f"Crater_{i:02d}"
        
        # Scale to make depression
        crater.scale[2] = 0.3
        
        # Apply dark crater material
        create_crater_material(crater)

def create_crater_material(obj):
    """Create dark crater material"""
    mat = bpy.data.materials.new(name="CraterMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    
    principled = nodes["Principled BSDF"]
    principled.inputs['Base Color'].default_value = (0.1, 0.1, 0.1, 1)
    principled.inputs['Roughness'].default_value = 0.95
    
    obj.data.materials.append(mat)

def create_lunar_rocks(num_rocks=30):
    """Add scattered lunar rocks"""
    
    for i in range(num_rocks):
        x = np.random.uniform(-45, 45)
        y = np.random.uniform(-45, 45)
        z = np.random.uniform(0.5, 2.0)
        
        # Create irregular rock shape
        bpy.ops.mesh.primitive_ico_sphere_add(radius=np.random.uniform(0.5, 1.5), location=(x, y, z))
        rock = bpy.context.active_object
        rock.name = f"Rock_{i:02d}"
        
        # Add random deformation
        bpy.ops.object.mode_set(mode='EDIT')
        bpy.ops.mesh.noise(factor=0.3)
        bpy.ops.object.mode_set(mode='OBJECT')
        
        # Apply rock material
        create_rock_material(rock)

def create_rock_material(obj):
    """Create realistic rock material"""
    mat = bpy.data.materials.new(name="RockMaterial")
    mat.use_nodes = True
    nodes = mat.node_tree.nodes
    
    principled = nodes["Principled BSDF"]
    principled.inputs['Base Color'].default_value = (0.25, 0.25, 0.25, 1)
    principled.inputs['Roughness'].default_value = 0.8
    
    obj.data.materials.append(mat)

def setup_lunar_lighting():
    """Setup realistic lunar lighting"""
    
    # Delete default light
    if "Light" in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects["Light"], do_unlink=True)
    
    # Add sun light (harsh directional lighting)
    bpy.ops.object.light_add(type='SUN', location=(50, 50, 100))
    sun = bpy.context.active_object
    sun.name = "LunarSun"
    sun.data.energy = 5.0
    sun.data.angle = 0.01  # Sharp shadows
    sun.rotation_euler = (0.3, 0.3, 0.8)

def setup_space_environment():
    """Setup black space environment"""
    
    # Set world background to black
    world = bpy.context.scene.world
    if world is None:
        world = bpy.data.worlds.new("LunarWorld")
        bpy.context.scene.world = world
    
    world.use_nodes = True
    bg_node = world.node_tree.nodes["Background"]
    bg_node.inputs['Color'].default_value = (0, 0, 0, 1)  # Pure black
    bg_node.inputs['Strength'].default_value = 0.0

def export_for_gazebo(filepath):
    """Export terrain optimized for Gazebo"""
    
    # Select terrain
    bpy.ops.object.select_all(action='DESELECT')
    terrain = bpy.data.objects.get("LunarTerrain")
    if terrain:
        terrain.select_set(True)
        bpy.context.view_layer.objects.active = terrain
        
        # Export as OBJ (Gazebo compatible)
        bpy.ops.export_scene.obj(
            filepath=filepath,
            use_selection=True,
            use_materials=True,
            use_smooth_groups=True,
            use_normals=True
        )
        
        print(f"✅ Terrain exported to: {filepath}")

def main():
    """Main terrain generation function"""
    
    print("🚀 Starting Professional Lunar Terrain Generation...")
    
    # Clear scene
    clear_scene()
    
    # Create terrain from image
    image_path = "/home/darshan/lunar/lunar_surface.jpg"
    terrain = create_lunar_terrain_from_image(image_path)
    
    if terrain:
        # Add features
        create_craters(terrain, num_craters=15)
        create_lunar_rocks(num_rocks=25)
        
        # Setup environment
        setup_lunar_lighting()
        setup_space_environment()
        
        # Export for Gazebo
        export_path = "/home/darshan/lunar/src/lunabot_simulation/meshes/lunar_terrain.obj"
        export_for_gazebo(export_path)
        
        # Save Blender file
        bpy.ops.wm.save_as_mainfile(filepath="/home/darshan/lunar/lunar_terrain.blend")
        
        print("🎉 Professional lunar terrain created!")
        print("📁 Blender file: /home/darshan/lunar/lunar_terrain.blend")
        print("🌍 Gazebo mesh: /home/darshan/lunar/src/lunabot_simulation/meshes/lunar_terrain.obj")

if __name__ == "__main__":
    main()

