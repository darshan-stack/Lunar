
import bpy
import bmesh

# Clear existing mesh objects
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)

# Import GLB
bpy.ops.import_scene.gltf(filepath="/home/darshan/lunar/24584_Curiosity_static.glb")

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
        filepath="/home/darshan/lunar/src/lunabot_simulation/models/custom_rover/meshes/rover.obj",
        use_selection=True,
        use_materials=True,
        use_smooth_groups=True,
        use_normals=True
    )
    
    print("✅ Rover model converted to OBJ")
else:
    print("❌ No mesh objects found in GLB file")
