"""Generate the PF2e heart tracker in Blender. All modeled coordinates are mm.

Run: blender --background --factory-startup --python build_tracker.py
Outputs STLs, a reusable Blender scene, dimensional checks, and a rendered preview.
"""
from pathlib import Path
import bpy
import bmesh
import json
import math
import struct
from mathutils import Vector

OUT = Path(__file__).resolve().parent
HEART_COUNT = 10
TRAY_LENGTH, TRAY_WIDTH, TRAY_HEIGHT = 150.0, 28.0, 10.0
PITCH = 14.0
HEART_WIDTH, HEART_HEIGHT, HEART_THICKNESS = 12.0, 12.0, 3.2
PEG_WIDTH, PEG_LENGTH = 3.2, 7.5
SOCKET_WIDTH, SOCKET_DEPTH = 3.6, 8.0
HEART_BEVEL = 0.22
SOCKET_BEVEL = 0.15


def activate(obj):
    bpy.ops.object.select_all(action='DESELECT')
    obj.select_set(True)
    bpy.context.view_layer.objects.active = obj


def apply(obj, modifier):
    activate(obj)
    bpy.ops.object.modifier_apply(modifier=modifier.name)


def bevel(obj, width, segments=3):
    modifier = obj.modifiers.new('Soft edges', 'BEVEL')
    modifier.width = width
    modifier.segments = segments
    modifier.limit_method = 'ANGLE'
    modifier.angle_limit = math.radians(35)
    apply(obj, modifier)


def box(name, dimensions, location):
    bpy.ops.mesh.primitive_cube_add(size=1, location=location)
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = dimensions
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return obj


def subtract(obj, cutter):
    modifier = obj.modifiers.new('Cut ' + cutter.name, 'BOOLEAN')
    modifier.operation = 'DIFFERENCE'
    modifier.solver = 'EXACT'
    modifier.object = cutter
    apply(obj, modifier)
    bpy.data.objects.remove(cutter, do_unlink=True)


def cubic(a, b, c, d, steps=64):
    points = []
    for i in range(steps + 1):
        t = i / steps
        points.append(tuple((1-t)**3*a[j] + 3*(1-t)**2*t*b[j]
                            + 3*(1-t)*t*t*c[j] + t**3*d[j] for j in range(2)))
    return points


def heart_outline():
    # One smooth half-heart; the lower tip is merged directly into the peg.
    left = cubic((0, 9.8), (-2.6, 13.6), (-6, 11.2), (-6, 8.4))
    left += cubic((-6, 8.4), (-6, 5.5), (-2.7, 2.6), (0, 0))[1:]
    ymax = max(y for x, y in left)
    left = [(x * HEART_WIDTH / 12, y * HEART_HEIGHT / ymax) for x, y in left]
    half = PEG_WIDTH / 2
    cropped = []
    for i, (x, y) in enumerate(left):
        if i and left[i-1][0] < -half <= x:
            px, py = left[i-1]
            t = (-half - px) / (x - px)
            cropped.append((-half, py + t*(y-py)))
            break
        cropped.append((x, y))
    return cropped + [(-half, -PEG_LENGTH), (half, -PEG_LENGTH)] + [
        (-x, y) for x, y in reversed(cropped[1:])]


def extrude_outline(name, points, thickness):
    n = len(points)
    vertices = [(x, y, z) for z in (0, thickness) for x, y in points]
    faces = [tuple(reversed(range(n))), tuple(range(n, n*2))]
    faces += [(i, (i+1) % n, (i+1) % n+n, i+n) for i in range(n)]
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(vertices, [], faces)
    mesh.update()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def make_heart():
    obj = extrude_outline('Heart pin - print flat', heart_outline(), HEART_THICKNESS)
    bevel(obj, HEART_BEVEL, 4)
    # Translate the print file to a positive-Y footprint and Z=0.
    for vertex in obj.data.vertices:
        vertex.co.y += PEG_LENGTH
    return obj


def make_tray(socket_width=SOCKET_WIDTH):
    tray = box('Tray - 10 sockets', (TRAY_LENGTH, TRAY_WIDTH, TRAY_HEIGHT),
               (0, 0, TRAY_HEIGHT/2))
    bevel(tray, 0.7, 4)
    for i in range(HEART_COUNT):
        x = (i-(HEART_COUNT-1)/2)*PITCH
        cutter = box('Socket', (socket_width, socket_width, SOCKET_DEPTH+2),
                     (x, 0, TRAY_HEIGHT-SOCKET_DEPTH+(SOCKET_DEPTH+2)/2))
        subtract(tray, cutter)
    bevel(tray, SOCKET_BEVEL, 1)
    return tray


def engrave(obj, label, x, y):
    curve = bpy.data.curves.new('Engraving ' + label, 'FONT')
    curve.body = label
    curve.size = 2.8
    curve.align_x = 'CENTER'
    curve.align_y = 'CENTER'
    curve.extrude = 0.6
    text = bpy.data.objects.new('Socket width ' + label, curve)
    bpy.context.collection.objects.link(text)
    text.location = (x, y, TRAY_HEIGHT-0.45)
    activate(text)
    bpy.ops.object.convert(target='MESH')
    subtract(obj, bpy.context.object)


def make_fit_test():
    obj = box('Fit test - socket widths in mm', (60, 22, TRAY_HEIGHT),
              (0, 0, TRAY_HEIGHT/2))
    bevel(obj, 0.7, 4)
    for i, width in enumerate((3.4, 3.5, 3.6, 3.7)):
        x = (i-1.5)*14
        cutter = box('Test socket', (width, width, SOCKET_DEPTH+2),
                     (x, 3, TRAY_HEIGHT-SOCKET_DEPTH+(SOCKET_DEPTH+2)/2))
        subtract(obj, cutter)
    bevel(obj, SOCKET_BEVEL, 1)
    for i, width in enumerate((3.4, 3.5, 3.6, 3.7)):
        engrave(obj, f'{width:.1f}', (i-1.5)*14, -5)
    return obj


def triangle_mesh(obj):
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.transform(bm, matrix=obj.matrix_world, verts=bm.verts)
    bmesh.ops.remove_doubles(bm, verts=list(bm.verts), dist=0.00001)
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=0.00001)
    bmesh.ops.triangulate(bm, faces=list(bm.faces))
    bmesh.ops.dissolve_degenerate(bm, edges=list(bm.edges), dist=0.00001)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.normal_update()
    return bm


def inspect_mesh(obj):
    bm = triangle_mesh(obj)
    remaining = set(bm.verts)
    components = 0
    while remaining:
        components += 1
        stack = [remaining.pop()]
        while stack:
            current = stack.pop()
            for edge in current.link_edges:
                other = edge.other_vert(current)
                if other in remaining:
                    remaining.remove(other)
                    stack.append(other)
    boundary = sum(not e.is_manifold for e in bm.edges)
    bad_area = sum(f.calc_area() < 1e-9 for f in bm.faces)
    volume = bm.calc_volume(signed=True)
    mins = [min(v.co[i] for v in bm.verts) for i in range(3)]
    maxs = [max(v.co[i] for v in bm.verts) for i in range(3)]
    result = dict(triangles=len(bm.faces), nonmanifold_edges=boundary,
                  zero_area_triangles=bad_area, connected_components=components,
                  volume_mm3=round(volume, 3), min_mm=[round(v, 4) for v in mins],
                  max_mm=[round(v, 4) for v in maxs],
                  size_mm=[round(b-a, 4) for a, b in zip(mins, maxs)])
    bm.free()
    assert boundary == 0 and bad_area == 0 and volume > 0, (obj.name, result)
    assert mins[2] >= -1e-5, (obj.name, 'Below print bed', mins)
    assert components == 1, (obj.name, 'Disconnected geometry', components)
    return result


def export_stl(path, objects):
    triangles = []
    for obj in objects:
        bm = triangle_mesh(obj)
        for face in bm.faces:
            normal = face.normal
            coordinates = [component for v in face.verts for component in v.co]
            triangles.append(struct.pack('<12fH', *normal, *coordinates, 0))
        bm.free()
    with path.open('wb') as file:
        file.write(b'PF2e heart tracker | units: millimeters'.ljust(80, b'\0'))
        file.write(struct.pack('<I', len(triangles)))
        file.writelines(triangles)


def material(name, color, roughness=0.4):
    mat = bpy.data.materials.new(name)
    mat.diffuse_color = (*color, 1)
    mat.use_nodes = True
    shader = mat.node_tree.nodes.get('Principled BSDF')
    shader.inputs['Base Color'].default_value = (*color, 1)
    shader.inputs['Roughness'].default_value = roughness
    return mat


def look_at(obj, point):
    obj.rotation_euler = (Vector(point)-obj.location).to_track_quat('-Z', 'Y').to_euler()


def clone(obj, name):
    new = obj.copy()
    new.data = obj.data.copy()
    new.name = name
    bpy.context.collection.objects.link(new)
    return new


def light(name, position, power, size):
    data = bpy.data.lights.new(name, 'AREA')
    data.energy = power
    data.shape = 'DISK'
    data.size = size
    obj = bpy.data.objects.new(name, data)
    bpy.context.collection.objects.link(obj)
    obj.location = position
    look_at(obj, (0, 0, 0))


def main():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)
    scene = bpy.context.scene
    scene.unit_settings.system = 'METRIC'
    scene.unit_settings.scale_length = 0.001
    scene.unit_settings.length_unit = 'MILLIMETERS'

    heart, tray, fit = make_heart(), make_tray(), make_fit_test()
    bpy.context.view_layer.update()
    checks = {name: inspect_mesh(obj) for name, obj in
              [('heart_pin', heart), ('tray_10_hearts', tray), ('peg_fit_test', fit)]}
    export_stl(OUT/'heart_pin.stl', [heart])
    export_stl(OUT/'tray_10_hearts.stl', [tray])
    export_stl(OUT/'peg_fit_test.stl', [fit])
    options = OUT/'tray-fit-options'
    options.mkdir(exist_ok=True)
    for width in (3.4, 3.5, 3.7):
        alternative = make_tray(width)
        bpy.context.view_layer.update()
        name = f'tray_socket_{width:.1f}mm'
        checks[name] = inspect_mesh(alternative)
        export_stl(options/(name+'.stl'), [alternative])
        bpy.data.objects.remove(alternative, do_unlink=True)
    ten = []
    for i in range(HEART_COUNT):
        obj = clone(heart, f'Print heart {i+1:02}')
        obj.location = ((i%5)*16, (i//5)*24, 0)
        ten.append(obj)
    bpy.context.view_layer.update()
    for obj in ten:
        inspect_mesh(obj)
    export_stl(OUT/'hearts_10_flat.stl', ten)
    checks['hearts_10_flat'] = dict(connected_components=10, layout='5 by 2',
                                   note='Ten intentionally separate solid heart pins.')
    for obj in ten:
        bpy.data.objects.remove(obj, do_unlink=True)

    # Fit is checked with the actual seated solids, using an exact intersection.
    seated = clone(heart, 'Seated geometry check')
    seated.rotation_euler.x = math.pi/2
    seated.location = (-4.5*PITCH, HEART_THICKNESS/2, TRAY_HEIGHT-SOCKET_DEPTH)
    bpy.context.view_layer.update()
    intersection = clone(tray, 'Interference check')
    mod = intersection.modifiers.new('Intersect actual seated heart', 'BOOLEAN')
    mod.operation, mod.solver, mod.object = 'INTERSECT', 'EXACT', seated
    apply(intersection, mod)
    bm = triangle_mesh(intersection)
    interference = abs(bm.calc_volume()) if bm.faces else 0
    bm.free()
    assert interference < 0.001, ('Peg collides with tray', interference)
    checks['assembly'] = dict(interference_volume_mm3=round(interference, 8),
                              socket_count=HEART_COUNT, socket_width_mm=SOCKET_WIDTH,
                              peg_width_mm=PEG_WIDTH,
                              clearance_per_side_mm=(SOCKET_WIDTH-PEG_WIDTH)/2,
                              socket_depth_mm=SOCKET_DEPTH,
                              floor_thickness_mm=TRAY_HEIGHT-SOCKET_DEPTH,
                              heart_center_spacing_mm=PITCH,
                              heart_gap_mm=PITCH-HEART_WIDTH)
    for obj in (seated, intersection):
        bpy.data.objects.remove(obj, do_unlink=True)
    (OUT/'mesh_checks.json').write_text(json.dumps(checks, indent=2)+'\n')
    print('TRACKER_MESH_CHECKS', json.dumps(checks), flush=True)

    red = material('Red PLA', (0.65, 0.018, 0.026), 0.31)
    charcoal = material('Charcoal PLA', (0.055, 0.066, 0.083), 0.47)
    for obj, mat in ((heart, red), (tray, charcoal), (fit, charcoal)):
        obj.data.materials.clear()
        obj.data.materials.append(mat)
        for polygon in obj.data.polygons:
            polygon.material_index = 0

    # Save source parts at a separate print-layout location in an organized collection.
    parts = bpy.data.collections.new('PRINT PARTS - millimeters')
    scene.collection.children.link(parts)
    for obj in (heart, fit):
        for collection in list(obj.users_collection):
            collection.objects.unlink(obj)
        parts.objects.link(obj)
    heart.location = (-20, 70, 0)
    fit.location = (30, 80, 0)
    parts.hide_render = True

    for i in range(HEART_COUNT):
        obj = clone(heart, f'Heart {i+1:02} - assembled')
        obj.rotation_euler.x = math.pi/2
        obj.location = ((i-4.5)*PITCH, HEART_THICKNESS/2,
                        TRAY_HEIGHT-SOCKET_DEPTH + (12 if i == 9 else 0))

    floor = box('Preview ground - not a print part', (2000, 2000, 1), (0, 0, -0.65))
    floor.data.materials.append(material('Warm studio', (0.75, 0.72, 0.66), 0.8))
    bpy.ops.object.camera_add(location=(110, -230, 155))
    camera = bpy.context.object
    camera.name = 'Preview camera'
    camera.data.type = 'ORTHO'
    camera.data.ortho_scale = 181
    camera.data.clip_end = 3000
    look_at(camera, (0, 0, 10))
    scene.camera = camera
    light('Key softbox', (-70, -90, 160), 400000, 100)
    light('Fill softbox', (80, -30, 90), 180000, 80)
    light('Rim softbox', (20, 90, 140), 300000, 80)
    scene.world.color = (0.3, 0.3, 0.3)
    scene.render.engine = 'CYCLES'
    scene.cycles.samples = 32
    scene.cycles.use_denoising = True
    scene.render.resolution_x = 1600
    scene.render.resolution_y = 850
    scene.render.resolution_percentage = 100
    scene.render.image_settings.file_format = 'PNG'
    scene.render.filepath = str(OUT/'assembled_preview.png')
    scene.view_settings.view_transform = 'AgX'
    bpy.context.preferences.filepaths.save_version = 0
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'health_tracker.blend'))
    bpy.ops.render.render(write_still=True)
    print('TRACKER_COMPLETE', str(OUT), flush=True)


if __name__ == '__main__':
    main()
