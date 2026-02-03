"""
Blender script to generate custom AUV mesh models (Version 3 - Origin-centered)

HOW TO USE:
1. Open Blender
2. Go to "Scripting" tab
3. Click "Open" and select this file
4. Click "Run Script" button (or Alt+P)
5. All AUV parts will be created at origin
6. Manually select and export each part as OBJ

IMPORTANT: All parts are centered at origin (0, 0, 0)
Position them in the scenario file using <compound_transform>

AUV Specifications:
- Length: 1.85m
- Diameter: 0.19m (radius: 0.095m)
- Weight: 48kg
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
    Main cylindrical hull with blunt front cap
    Centered at origin, extends along X axis
    """
    # Cylinder (1.62m)
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=64,
        radius=0.095,
        depth=1.62,
        location=(0, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    cylinder = bpy.context.active_object

    # Front cap - very blunt
    bpy.ops.mesh.primitive_uv_sphere_add(
        segments=64,
        ring_count=16,
        radius=0.095,
        location=(0.81, 0, 0),
        rotation=(0, math.pi/2, 0)
    )
    front_cap = bpy.context.active_object

    # Flatten for blunt nose
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.transform.resize(value=(0.1, 1, 1))
    bpy.ops.object.mode_set(mode='OBJECT')

    # Cut and position
    front_cap.location = (0.815, 0, 0)
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.mesh.bisect(plane_co=(0.81, 0, 0), plane_no=(-1, 0, 0), clear_outer=True)
    bpy.ops.object.mode_set(mode='OBJECT')

    # Join
    cylinder.select_set(True)
    front_cap.select_set(True)
    bpy.context.view_layer.objects.active = cylinder
    bpy.ops.object.join()

    hull = bpy.context.active_object
    hull.name = "MainHull"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {hull.name} (~1.63m, centered at origin)")
    return hull

def create_tail_cone():
    """
    Tail cone - centered at origin
    Position in scenario: ~(-0.92, 0, 0)
    """
    bpy.ops.mesh.primitive_cone_add(
        vertices=64,
        radius1=0.095,
        radius2=0.065,
        depth=0.22,
        location=(0, 0, 0),  # Origin
        rotation=(0, math.pi/2, 0)
    )
    tail = bpy.context.active_object
    tail.name = "TailCone"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {tail.name} (centered at origin)")
    return tail

def create_control_fin():
    """
    Control fin - centered at origin
    Position in scenario: ~(-0.7, 0, 0) + rotation for placement
    """
    verts = [
        (-0.075, 0, 0.005), (-0.075, 0, -0.005),
        (0.075, 0, 0.005), (0.075, 0, -0.005),
        (-0.04, 0.12, 0.005), (-0.04, 0.12, -0.005),
        (0.04, 0.12, 0.005), (0.04, 0.12, -0.005),
    ]

    faces = [
        (0, 1, 3, 2), (4, 5, 7, 6),
        (0, 1, 5, 4), (2, 3, 7, 6),
        (0, 2, 6, 4), (1, 3, 7, 5),
    ]

    mesh = bpy.data.meshes.new("FinMesh")
    mesh.from_pydata(verts, [], faces)
    mesh.update()

    fin = bpy.data.objects.new("ControlFin", mesh)
    bpy.context.collection.objects.link(fin)
    bpy.context.view_layer.objects.active = fin
    fin.select_set(True)

    # Center at origin
    fin.location = (0, 0, 0)
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {fin.name} (centered at origin)")
    return fin

def create_propeller_duct():
    """
    Propeller duct - centered at origin
    Position in scenario: ~(-1.03, 0, 0)
    """
    # Outer
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=64,
        radius=0.09,
        depth=0.1,
        location=(0, 0, 0),  # Origin
        rotation=(0, math.pi/2, 0)
    )
    outer = bpy.context.active_object

    # Inner
    bpy.ops.mesh.primitive_cylinder_add(
        vertices=64,
        radius=0.08,
        depth=0.12,
        location=(0, 0, 0),  # Origin
        rotation=(0, math.pi/2, 0)
    )
    inner = bpy.context.active_object

    # Boolean
    outer.select_set(True)
    bpy.context.view_layer.objects.active = outer
    bool_mod = outer.modifiers.new(name="Boolean", type='BOOLEAN')
    bool_mod.operation = 'DIFFERENCE'
    bool_mod.object = inner
    bpy.ops.object.modifier_apply(modifier="Boolean")
    bpy.data.objects.remove(inner, do_unlink=True)

    outer.name = "PropellerDuct"
    bpy.ops.object.shade_smooth()

    print(f"✓ Created: {outer.name} (centered at origin)")
    return outer

def main():
    """Generate all AUV mesh models"""
    print("\n" + "=" * 60)
    print("Creating Custom AUV Mesh Models (Origin-Centered)")
    print("=" * 60)

    clear_scene()

    print("\n[1/4] Creating main hull...")
    hull = create_main_hull()

    print("[2/4] Creating tail cone...")
    tail = create_tail_cone()

    print("[3/4] Creating control fin...")
    fin = create_control_fin()

    print("[4/4] Creating propeller duct...")
    duct = create_propeller_duct()

    bpy.ops.object.select_all(action='DESELECT')

    print("\n" + "=" * 60)
    print("All parts created at ORIGIN!")
    print("=" * 60)
    print("\nIMPORTANT: Position in scenario file using:")
    print("  <compound_transform xyz=\"X Y Z\"/>")
    print("\nSuggested positions:")
    print("  MainHull:       xyz=\"0.0 0.0 0.0\"")
    print("  TailCone:       xyz=\"-0.92 0.0 0.0\"")
    print("  ControlFin:     xyz=\"-0.7 0.0 0.0\" + rotation")
    print("  PropellerDuct:  xyz=\"-1.03 0.0 0.0\"")
    print("\n" + "=" * 60)
    print("EXPORT:")
    print("=" * 60)
    print("  MainHull → custom_auv_hull.obj")
    print("  TailCone → custom_auv_tail.obj")
    print("  ControlFin → custom_auv_fin.obj")
    print("  PropellerDuct → custom_auv_duct.obj")

if __name__ == "__main__":
    main()