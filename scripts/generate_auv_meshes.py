"""
Blender script to generate custom AUV mesh models

HOW TO USE:
1. Open Blender
2. Go to "Scripting" tab
3. Click "Open" and select this file
4. Click "Run Script" button (or Alt+P)
5. All AUV parts will be created in the scene
6. Manually select and export each part as OBJ

EXPORT INSTRUCTIONS:
- Select object → File → Export → Wavefront (.obj)
- Export settings:
  - Forward: X Forward
  - Up: Z Up
  - Uncheck "Export Materials"

File names to use when exporting:
- MainHull → custom_auv_hull.obj
- NoseCone → custom_auv_nose.obj
- TailCone → custom_auv_tail.obj
- ControlFin → custom_auv_fin.obj
- PropellerDuct → custom_auv_duct.obj

AUV Specifications:
- Length: 1.85m
- Diameter: 0.19m (radius: 0.095m)
- Weight: 48kg
- CRP (Counter-Rotating Propeller) at stern
- Four control fins (dorsal, ventral, port, starboard)
"""

import bpy
import bmesh
import math

def clear_scene():
    """Clear all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    print("✓ Scene cleared")

def create_main_hull():
    """
    Create main cylindrical hull
    Length: 1.3m (center section)
    Diameter: 0.19m
    Position: Center (0, 0, 0)
    """
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32,
        radius=0.095,
        depth=1.3,
        location=(0, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    hull = bpy.context.active_object
    hull.name = "MainHull"

    # Add subdivision for smoother surface
    bpy.ops.object.modifier_add(type='SUBSURF')
    hull.modifiers["Subdivision"].levels = 2
    bpy.ops.object.modifier_apply(modifier="Subdivision")
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {hull.name}")
    return hull

def create_nose_cone():
    """
    Create streamlined nose cone
    Length: 0.35m
    Base diameter: 0.19m
    Position: Front (0.825, 0, 0)
    """
    bpy.ops.mesh.primitive_cone_add(
        vertices=32,
        radius1=0.095,
        radius2=0.01,
        depth=0.35,
        location=(0.825, 0, 0),
        rotation=(0, -math.pi/2, 0)
    )
    nose = bpy.context.active_object
    nose.name = "NoseCone"

    # Smooth the cone
    bpy.ops.object.modifier_add(type='SUBSURF')
    nose.modifiers["Subdivision"].levels = 2
    bpy.ops.object.modifier_apply(modifier="Subdivision")
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {nose.name}")
    return nose

def create_tail_cone():
    """
    Create tail section with propeller housing
    Length: 0.2m
    Base diameter: 0.19m
    End diameter: 0.12m
    Position: Rear (-0.825, 0, 0)
    """
    bpy.ops.mesh.primitive_cone_add(
        vertices=32,
        radius1=0.095,
        radius2=0.06,
        depth=0.2,
        location=(-0.825, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    tail = bpy.context.active_object
    tail.name = "TailCone"

    # Smooth the tail
    bpy.ops.object.modifier_add(type='SUBSURF')
    tail.modifiers["Subdivision"].levels = 2
    bpy.ops.object.modifier_apply(modifier="Subdivision")
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {tail.name}")
    return tail

def create_control_fin():
    """
    Create control fin (rudder/elevator)
    Root chord: 0.15m
    Tip chord: 0.08m
    Span: 0.12m
    Thickness: 0.01m
    Position: At tail section (-0.7, 0, 0) for dorsal fin
    """
    # Create a tapered wing-like fin
    verts = [
        # Root leading edge
        (-0.075, 0, 0.005),
        (-0.075, 0, -0.005),
        # Root trailing edge
        (0.075, 0, 0.005),
        (0.075, 0, -0.005),
        # Tip leading edge
        (-0.04, 0.12, 0.005),
        (-0.04, 0.12, -0.005),
        # Tip trailing edge
        (0.04, 0.12, 0.005),
        (0.04, 0.12, -0.005),
    ]

    faces = [
        # Root
        (0, 1, 3, 2),
        # Tip
        (4, 5, 7, 6),
        # Leading edge
        (0, 1, 5, 4),
        # Trailing edge
        (2, 3, 7, 6),
        # Top surface
        (0, 2, 6, 4),
        # Bottom surface
        (1, 3, 7, 5),
    ]

    mesh = bpy.data.meshes.new("FinMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    fin = bpy.data.objects.new("ControlFin", mesh)
    bpy.context.collection.objects.link(fin)
    bpy.context.view_layer.objects.active = fin
    fin.select_set(True)

    # Position at tail
    fin.location = (-0.7, 0, 0)

    # Add edge split for sharp edges
    bpy.ops.object.modifier_add(type='EDGE_SPLIT')
    bpy.ops.object.modifier_apply(modifier="EdgeSplit")
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {fin.name}")
    return fin

def create_propeller_duct():
    """
    Create duct for CRP system (optional)
    Inner diameter: 0.16m
    Outer diameter: 0.18m
    Length: 0.1m
    Position: At rear (-0.975, 0, 0)
    """
    # Outer cylinder
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32,
        radius=0.09,
        depth=0.1,
        location=(-0.975, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    outer = bpy.context.active_object

    # Inner cylinder (subtract)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=32,
        radius=0.08,
        depth=0.12,
        location=(-0.975, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    inner = bpy.context.active_object

    # Use boolean modifier to create duct
    outer.select_set(True)
    bpy.context.view_layer.objects.active = outer
    bool_mod = outer.modifiers.new(name="Boolean", type='BOOLEAN')
    bool_mod.operation = 'DIFFERENCE'
    bool_mod.object = inner
    bpy.ops.object.modifier_apply(modifier="Boolean")

    # Delete inner cylinder
    bpy.data.objects.remove(inner, do_unlink=True)

    outer.name = "PropellerDuct"
    print(f"✓ Created: {outer.name}")
    return outer

def main():
    """Generate all AUV mesh models in the scene"""
    print("\n" + "=" * 60)
    print("Creating Custom AUV Mesh Models")
    print("=" * 60)

    # Clear scene
    clear_scene()

    # Create all parts (they will be positioned correctly in the scene)
    print("\n[1/5] Creating main hull...")
    hull = create_main_hull()

    print("[2/5] Creating nose cone...")
    nose = create_nose_cone()

    print("[3/5] Creating tail cone...")
    tail = create_tail_cone()

    print("[4/5] Creating control fin...")
    fin = create_control_fin()

    print("[5/5] Creating propeller duct...")
    duct = create_propeller_duct()

    # Deselect all
    bpy.ops.object.select_all(action='DESELECT')

    print("\n" + "=" * 60)
    print("All parts created in the scene!")
    print("=" * 60)
    print("\nCreated objects:")
    print("  ✓ MainHull - Center body")
    print("  ✓ NoseCone - Front cone")
    print("  ✓ TailCone - Rear cone")
    print("  ✓ ControlFin - Dorsal fin (use for all 4 fins)")
    print("  ✓ PropellerDuct - Thruster duct")

    print("\n" + "=" * 60)
    print("EXPORT INSTRUCTIONS:")
    print("=" * 60)
    print("1. Select each object in the Outliner")
    print("2. File → Export → Wavefront (.obj)")
    print("3. Set export options:")
    print("   - Forward: X Forward")
    print("   - Up: Z Up")
    print("   - Uncheck 'Export Materials'")
    print("\nFile names to use:")
    print("   MainHull → custom_auv_hull.obj")
    print("   NoseCone → custom_auv_nose.obj")
    print("   TailCone → custom_auv_tail.obj")
    print("   ControlFin → custom_auv_fin.obj")
    print("   PropellerDuct → custom_auv_duct.obj")
    print("\nTotal Length: ~1.85m")
    print("Diameter: 0.19m")

if __name__ == "__main__":
    main()