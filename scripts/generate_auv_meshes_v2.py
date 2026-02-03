"""
Blender script to generate custom AUV mesh models (Version 2 - Clean connections)

HOW TO USE:
1. Open Blender
2. Go to "Scripting" tab
3. Click "Open" and select this file
4. Click "Run Script" button (or Alt+P)
5. All AUV parts will be created in the scene
6. Manually select and export each part as OBJ

CHANGES FROM V1:
- Removed subdivision to avoid wavy connections
- Cleaner, simpler geometry
- Better alignment between parts

AUV Specifications:
- Length: 1.85m
- Diameter: 0.19m (radius: 0.095m)
- Weight: 48kg
- CRP (Counter-Rotating Propeller) at stern
"""

import bpy
import math

def clear_scene():
    """Clear all objects from the scene"""
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    print("✓ Scene cleared")

def create_main_hull():
    """
    Create main cylindrical hull (clean cylinder, no subdivision)
    Length: 1.3m (center section)
    Diameter: 0.19m
    """
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=64,  # Higher vertex count for smooth circle
        radius=0.095,
        depth=1.3,
        location=(0, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    hull = bpy.context.active_object
    hull.name = "MainHull"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {hull.name}")
    return hull

def create_nose_cone():
    """
    Create blunt nose cone (no subdivision)
    Length: 0.35m
    Base diameter: 0.19m
    Tip diameter: 0.06m (blunt, not sharp)
    """
    bpy.ops.mesh.primitive_cone_add(
        vertices=64,
        radius1=0.095,
        radius2=0.03,  # Much larger for blunt nose
        depth=0.35,
        location=(0.825, 0, 0),
        rotation=(0, -math.pi/2, 0)
    )
    nose = bpy.context.active_object
    nose.name = "NoseCone"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {nose.name}")
    return nose

def create_tail_cone():
    """
    Create tail section (no subdivision)
    Length: 0.2m
    Base diameter: 0.19m
    End diameter: 0.12m
    """
    bpy.ops.mesh.primitive_cone_add(
        vertices=64,
        radius1=0.095,
        radius2=0.06,
        depth=0.2,
        location=(-0.825, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    tail = bpy.context.active_object
    tail.name = "TailCone"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {tail.name}")
    return tail

def create_control_fin():
    """
    Create control fin (simple, clean)
    """
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
        (0, 1, 3, 2),  # Root
        (4, 5, 7, 6),  # Tip
        (0, 1, 5, 4),  # Leading edge
        (2, 3, 7, 6),  # Trailing edge
        (0, 2, 6, 4),  # Top surface
        (1, 3, 7, 5),  # Bottom surface
    ]

    mesh = bpy.data.meshes.new("FinMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    fin = bpy.data.objects.new("ControlFin", mesh)
    bpy.context.collection.objects.link(fin)
    bpy.context.view_layer.objects.active = fin
    fin.select_set(True)
    fin.location = (-0.7, 0, 0)

    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {fin.name}")
    return fin

def create_propeller_duct():
    """
    Create duct for CRP system (simple rings)
    """
    # Outer cylinder
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=64,
        radius=0.09,
        depth=0.1,
        location=(-0.975, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    outer = bpy.context.active_object

    # Inner cylinder
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=64,
        radius=0.08,
        depth=0.12,
        location=(-0.975, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    inner = bpy.context.active_object

    # Boolean operation
    outer.select_set(True)
    bpy.context.view_layer.objects.active = outer
    bool_mod = outer.modifiers.new(name="Boolean", type='BOOLEAN')
    bool_mod.operation = 'DIFFERENCE'
    bool_mod.object = inner
    bpy.ops.object.modifier_apply(modifier="Boolean")

    bpy.data.objects.remove(inner, do_unlink=True)

    outer.name = "PropellerDuct"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {outer.name}")
    return outer

def main():
    """Generate all AUV mesh models"""
    print("\n" + "=" * 60)
    print("Creating Custom AUV Mesh Models (V2 - Clean)")
    print("=" * 60)

    clear_scene()

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

    bpy.ops.object.select_all(action='DESELECT')

    print("\n" + "=" * 60)
    print("All parts created!")
    print("=" * 60)
    print("\nCreated objects (without subdivision - clean connections):")
    print("  ✓ MainHull - Smooth cylinder")
    print("  ✓ NoseCone - Smooth cone")
    print("  ✓ TailCone - Smooth cone")
    print("  ✓ ControlFin - Fin shape")
    print("  ✓ PropellerDuct - Ring duct")
    print("\nTotal Length: ~1.85m")
    print("Diameter: 0.19m")

if __name__ == "__main__":
    main()