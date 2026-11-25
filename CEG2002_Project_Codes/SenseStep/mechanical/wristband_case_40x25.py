"""40 x 25 mm wristband enclosure for two boards and an external battery.

Run with Python 3.10+ to export two STLs and one pre-oriented 3MF plate.
All dimensions below describe the assembled case; export orients each part.
"""

from collections import Counter, defaultdict
from itertools import product
from math import sqrt
from pathlib import Path
import struct
from xml.etree import ElementTree as XML
from zipfile import ZIP_DEFLATED, ZipFile


LENGTH = 40.0
WIDTH = 25.0
HEIGHT = 9.4
SEAM = 3.35
ROOF = 8.1
FLOOR = 1.6
CORNER_RADIUS = 1.3

USB = (24.0, 36.3)
ANTENNA_SLOT = (25.5, 28.4)
# Center the opening under the two battery pads on the standard XIAO PCB.
BATTERY_WIRE_POCKET = (31.5, 37.5, 6.5, 14.5, 0.8, FLOOR)
BATTERY_WIRE_HOLE = (33.2, 35.8, 7.9, 12.3, 0, BATTERY_WIRE_POCKET[4])
BATTERY_WIRE_RADIUS = 1.3
LATCH = (8.5, 16.5)
MOTOR_SIDE_RELIEF = (0.65, 21.1)
PART_ENVELOPES = {
    "Motor PCB": (0.9, 20.9, 0.5, 24.5, 1.85, 2.95),
    "Motor components": (1.25, 20.6, 1.3, 23.7, 2.95, 6.85),
    "Motor coil leads": (15.6, 20.8, 0.4, 4.0, 2.95, 4.15),
    "XIAO PCB": (21.25, 39.05, 0.9, 21.9, 1.95, 3.15),
    "XIAO components and solder": (21.5, 38.65, 1.6, 21.9, 3.15, 6.55),
    "USB-C receptacle": (25.55, 34.75, 0.45, 7.7, 3.0, 6.5),
}


def box(x0, x1, y0, y1, z0, z1):
    return (x0, x1, y0, y1, z0, z1)


def side(x0, x1, inset0, inset1, z0, z1, far):
    """A box measured inward from one of the long sides."""
    if far:
        return box(x0, x1, WIDTH - inset1, WIDTH - inset0, z0, z1)
    return box(x0, x1, inset0, inset1, z0, z1)


def end(y0, y1, inset0, inset1, z0, z1, right):
    """A box measured inward from one of the short ends."""
    if right:
        return box(LENGTH - inset1, LENGTH - inset0, y0, y1, z0, z1)
    return box(inset0, inset1, y0, y1, z0, z1)


def case_recipe():
    """Ordered additions and cuts for the electronics tray and snap lid."""
    tray = [
        (True, box(0, LENGTH, 0, WIDTH, 0, SEAM)),
        (False, box(0.65, LENGTH - 0.65, 0.85, WIDTH - 0.85, FLOOR, SEAM)),
        (False, side(*USB, 0, 0.85, 2.4, SEAM, False)),
        # The pocket lets the underside solder joints and insulated leads sit
        # below the mounting surface. The wire ends feed through the tray.
        (False, BATTERY_WIRE_POCKET),
        (False, BATTERY_WIRE_HOLE),
        (False, side(*ANTENNA_SLOT, 0, 0.85, 2.65, SEAM, True)),
    ]
    for far in (False, True):
        # Slots clear the 24 mm PCB across the 25 mm case while retaining full
        # wall thickness above the board and a full-width floor underneath it.
        tray.append((False, side(*MOTOR_SIDE_RELIEF, 0, 0.85, FLOOR, SEAM, far)))
    for right in (False, True):
        tray.extend((
            (False, end(LATCH[0] - 0.25, LATCH[1] + 0.25, 0, 0.68, 0.68, SEAM, right)),
            (False, end(LATCH[0] - 0.25, LATCH[1] + 0.25, 0, 1.20, 0.81, 1.30, right)),
            (False, end(LATCH[0] + 1.0, LATCH[1] - 1.0, 0, 0.48, 0.28, 0.73, right)),
        ))

    lid = [
        (True, box(0, LENGTH, 0, WIDTH, SEAM, HEIGHT)),
        (False, box(1.10, LENGTH - 1.10, 1.05, WIDTH - 1.05, SEAM, ROOF)),
        (False, side(*USB, 0, 1.05, SEAM, 7.1, False)),
        (False, side(15.2, MOTOR_SIDE_RELIEF[1], 0, 1.05, SEAM, 4.45, False)),
        # Open the antenna slot to the seam so the coax can be laid in.
        (False, side(*ANTENNA_SLOT, 0, 1.05, SEAM, 5.25, True)),
    ]
    for right in (False, True):
        lid.extend((
            (False, end(LATCH[0] - 0.9, LATCH[0], 0, 1.1, SEAM, 7.5, right)),
            (False, end(LATCH[1], LATCH[1] + 0.9, 0, 1.1, SEAM, 7.5, right)),
            (False, end(*LATCH, 0, 0.06, SEAM, 7.3, right)),
            (True, end(*LATCH, 0.06, 0.60, 0.70, SEAM, right)),
            (True, end(*LATCH, 0.60, 0.84, 0.90, 1.22, right)),
        ))
    return {"Electronics tray": tray, "Snap lid": lid}


def axis_grid(recipe, axis, extent=None):
    stops = {round(bounds[index], 5) for _, bounds in recipe for index in (axis * 2, axis * 2 + 1)}
    if extent is not None:
        for step in range(9):
            value = CORNER_RADIUS * step / 8
            stops.update((round(value, 5), round(extent - value, 5)))
        if (False, BATTERY_WIRE_HOLE) in recipe:
            lo, hi = BATTERY_WIRE_HOLE[axis * 2:axis * 2 + 2]
            for step in range(7):
                value = BATTERY_WIRE_RADIUS * step / 6
                stops.update((round(lo + value, 5), round(hi - value, 5)))
    elif min(stops) == 0:
        stops.update((0.25, 0.5, 0.68))
    elif max(stops) == HEIGHT:
        stops.update(round(HEIGHT - value, 5) for value in (0.25, 0.5, 0.68))
    return sorted(stops)


def rounded_xy(x, y, x0, x1, y0, y1, r):
    """Map each corner square onto a quarter circle."""
    cx = min(max(x, x0 + r), x1 - r)
    cy = min(max(y, y0 + r), y1 - r)
    dx, dy = x - cx, y - cy
    return (cx + dx * sqrt(1 - 0.5 * (dy / r) ** 2),
            cy + dy * sqrt(1 - 0.5 * (dx / r) ** 2))


def shape_point(x, y, z, part):
    x0, x1, y0, y1, _, hole_top = BATTERY_WIRE_HOLE
    if part == "Electronics tray" and z <= hole_top and x0 <= x <= x1 and y0 <= y <= y1:
        # Keep the full opening size through the tray's outside bevel.
        x, y = rounded_xy(x, y, x0, x1, y0, y1, BATTERY_WIRE_RADIUS)
        return (round(x, 5), round(y, 5), round(z, 5))

    # Apply the same outer corner mapping to both mating parts.
    x, y = rounded_xy(x, y, 0, LENGTH, 0, WIDTH, CORNER_RADIUS)

    depth = z if part == "Electronics tray" else HEIGHT - z if part == "Snap lid" else 2
    inset = 0.55 * max(0, 1 - depth / 0.68)
    x = inset + x * (1 - 2 * inset / LENGTH)
    y = inset + y * (1 - 2 * inset / WIDTH)

    # Taper the lower edge of each hook to deflect it outward on insertion.
    if part == "Snap lid" and z == 0.9 and LATCH[0] <= y <= LATCH[1]:
        near = min(x, LENGTH - x)
        if 0.59999 <= near <= 0.84001:
            ramp = 0.60 + (near - 0.60) * (0.04 / 0.24)
            x = ramp if x <= LENGTH / 2 else LENGTH - ramp
    return (round(x, 5), round(y, 5), round(z, 5))


# Each quad is ordered counterclockwise when viewed from outside the solid.
FACES = (
    ((-1, 0, 0), ((0, 0, 0), (0, 0, 1), (0, 1, 1), (0, 1, 0))),
    ((1, 0, 0), ((1, 0, 0), (1, 1, 0), (1, 1, 1), (1, 0, 1))),
    ((0, -1, 0), ((0, 0, 0), (1, 0, 0), (1, 0, 1), (0, 0, 1))),
    ((0, 1, 0), ((0, 1, 0), (0, 1, 1), (1, 1, 1), (1, 1, 0))),
    ((0, 0, -1), ((0, 0, 0), (0, 1, 0), (1, 1, 0), (1, 0, 0))),
    ((0, 0, 1), ((0, 0, 1), (1, 0, 1), (1, 1, 1), (0, 1, 1))),
)


def make_mesh(part, recipe):
    axes = [axis_grid(recipe, axis, extent) for axis, extent in enumerate((LENGTH, WIDTH, None))]
    indexes = [{value: index for index, value in enumerate(axis)} for axis in axes]
    occupied = set()
    for add, bounds in recipe:
        cells = product(*(
            range(indexes[axis][round(bounds[axis * 2], 5)],
                  indexes[axis][round(bounds[axis * 2 + 1], 5)])
            for axis in range(3)
        ))
        if add:
            occupied.update(cells)
        else:
            occupied.difference_update(cells)

    vertices = []
    vertex_ids = {}
    triangles = []
    for cell in sorted(occupied):
        for delta, corners in FACES:
            neighbor = tuple(i + d for i, d in zip(cell, delta))
            if neighbor in occupied:
                continue
            quad = []
            for corner in corners:
                vertex = tuple(axis[i + d] for axis, i, d in zip(axes, cell, corner))
                if vertex not in vertex_ids:
                    vertex_ids[vertex] = len(vertices)
                    vertices.append(shape_point(*vertex, part))
                quad.append(vertex_ids[vertex])
            a, b, c, d = quad
            triangles.extend(((a, b, c), (a, c, d)))
    return vertices, triangles


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def normal_vector(a, b, c):
    return cross(tuple(y - x for x, y in zip(a, b)), tuple(y - x for x, y in zip(a, c)))


def validate_mesh(vertices, triangles):
    directed = Counter()
    neighbors = defaultdict(set)
    volume = 0
    for a, b, c in triangles:
        normal = normal_vector(vertices[a], vertices[b], vertices[c])
        assert sum(value * value for value in normal) > 1e-16, (vertices[a], vertices[b], vertices[c])
        volume += sum(x * n for x, n in zip(vertices[a], normal)) / 6
        for start, end in ((a, b), (b, c), (c, a)):
            directed[start, end] += 1
            neighbors[start].add(end)
    assert len(vertices) == len(set(vertices)), "Duplicate vertices"
    assert all(count == directed[end, start] == 1 for (start, end), count in directed.items()), "Open edge"
    visited, pending = {0}, [0]
    while pending:
        for vertex in neighbors[pending.pop()] - visited:
            visited.add(vertex)
            pending.append(vertex)
    assert len(visited) == len(vertices) and volume > 0, "Expected one outward-facing solid"
    return volume


def check_layout(recipes):
    """Check every grid cell against the part and component envelopes."""
    envelopes = [(True, bounds) for bounds in PART_ENVELOPES.values()]
    geometry = [op for recipe in recipes.values() for op in recipe] + envelopes
    axes = [axis_grid(geometry, axis) for axis in range(3)]
    indexes = [{value: i for i, value in enumerate(axis)} for axis in axes]

    def cells(bounds):
        return set(product(*(
            range(indexes[axis][round(bounds[2 * axis], 5)],
                  indexes[axis][round(bounds[2 * axis + 1], 5)])
            for axis in range(3)
        )))

    assembly = set()
    for part, recipe in recipes.items():
        solid = set()
        for add, bounds in recipe:
            if add:
                solid.update(cells(bounds))
            else:
                solid.difference_update(cells(bounds))
        overlap = assembly & solid
        if overlap:
            x, y, z = min(overlap)
            raise ValueError(f"{part} intersects another printed part at {(axes[0][x], axes[1][y], axes[2][z])}")
        assembly.update(solid)
    for component, bounds in PART_ENVELOPES.items():
        assert assembly.isdisjoint(cells(bounds)), f"{component} intersects the case"


def print_orientation(part, vertices):
    if part == "Snap lid":
        return [(x, round(WIDTH - y, 5), round(HEIGHT - z, 5)) for x, y, z in vertices]
    base = min(z for _, _, z in vertices)
    return [(x, y, round(z - base, 5)) for x, y, z in vertices]


def write_stl(path, vertices, triangles):
    with path.open("wb") as stream:
        stream.write(path.stem.encode("ascii").ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        for face in triangles:
            a, b, c = (vertices[i] for i in face)
            normal = normal_vector(a, b, c)
            magnitude = sqrt(sum(value * value for value in normal))
            stream.write(struct.pack("<12fH", *(value / magnitude for value in normal), *a, *b, *c, 0))


def write_plate(path, meshes):
    namespace = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
    model = XML.Element("model", {"unit": "millimeter", "xml:lang": "en-US", "xmlns": namespace})
    resources = XML.SubElement(model, "resources")
    build = XML.SubElement(model, "build")
    for i, (part, (vertices, triangles)) in enumerate(meshes.items(), 1):
        obj = XML.SubElement(resources, "object", {"id": str(i), "type": "model", "name": part})
        mesh = XML.SubElement(obj, "mesh")
        points = XML.SubElement(mesh, "vertices")
        for x, y, z in vertices:
            XML.SubElement(points, "vertex", {"x": str(x), "y": str(y), "z": str(z)})
        faces = XML.SubElement(mesh, "triangles")
        for a, b, c in triangles:
            XML.SubElement(faces, "triangle", {"v1": str(a), "v2": str(b), "v3": str(c)})
        x, y = (8 + (i - 1) * (LENGTH + 7), 8)
        XML.SubElement(build, "item", {"objectid": str(i), "transform": f"1 0 0 0 1 0 0 0 1 {x} {y} 0"})
    types = """<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
</Types>"""
    relations = """<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/>
</Relationships>"""
    with ZipFile(path, "w", ZIP_DEFLATED) as archive:
        archive.writestr("[Content_Types].xml", types)
        archive.writestr("_rels/.rels", relations)
        archive.writestr("3D/3dmodel.model", XML.tostring(model, encoding="utf-8", xml_declaration=True))


def main():
    destination = Path(__file__).resolve().parent
    meshes = {}
    recipes = case_recipe()
    check_layout(recipes)
    for part, recipe in recipes.items():
        vertices, triangles = make_mesh(part, recipe)
        vertices = print_orientation(part, vertices)
        volume = validate_mesh(vertices, triangles)
        filename = part.lower().replace(" ", "_") + ".stl"
        write_stl(destination / filename, vertices, triangles)
        meshes[part] = (vertices, triangles)
        print(f"{part}: {len(triangles):,} triangles, {volume / 1000:.2f} cm³")
    write_plate(destination / "wristband_case_40x25.3mf", meshes)


if __name__ == "__main__":
    main()
