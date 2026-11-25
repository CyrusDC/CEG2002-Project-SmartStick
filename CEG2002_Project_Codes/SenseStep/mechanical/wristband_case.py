"""Parametric wristband enclosure in millimetres. Run with Python 3.10+.

Exports the three parts as STLs and as one pre-oriented 3MF plate.
All dimensions below describe the assembled case; export orients each part.
"""

from collections import Counter, defaultdict
from itertools import product
from math import sqrt
from pathlib import Path
import struct
from xml.etree import ElementTree as XML
from zipfile import ZIP_DEFLATED, ZipFile


LENGTH = 58.0
WIDTH = 37.6
HEIGHT = 20.8
SEAM = 14.0
ROOF = 19.4
FLOOR = 1.6
DIVIDER_BOTTOM = 12.6
DIVIDER_TOP = 13.8
CORNER_RADIUS = 3.6

USB = (34.8, 47.2)
ANTENNA_SLOT = (41.5, 44.7)
LATCH = (16.5, 25.5)
DIVIDER_STOPS = ((6.0, 9.4), (52.1, 54.3))
PART_ENVELOPES = {
    "LiPo": (2.4, 52.4, 2.8, 34.8, FLOOR, 11.6),
    "XIAO PCB": (32.1, 49.9, 0.9, 21.9, 14.15, 15.35),
    "XIAO components and solder": (32.1, 49.9, 1.6, 21.9, 15.35, 18.65),
    "USB-C receptacle": (36.4, 45.6, 0.45, 7.7, 15.15, 18.65),
    "Motor module": (6.4, 30.4, 4.0, 24.0, 14.05, 19.05),
    "Plug bay": (46.5, 55.5, 24.0, 34.5, DIVIDER_TOP, ROOF),
}


def box(x0, x1, y0, y1, z0, z1):
    return (x0, x1, y0, y1, z0, z1)


def side(x0, x1, inset0, inset1, z0, z1, far):
    """A box measured inward from one of the long sides."""
    if far:
        return box(x0, x1, WIDTH - inset1, WIDTH - inset0, z0, z1)
    return box(x0, x1, inset0, inset1, z0, z1)


def case_recipe():
    """Ordered additions and cuts for the tub, divider and upside-down lid."""
    tub = [(True, box(0, LENGTH, 0, WIDTH, 0, SEAM))]
    tub += [
        (False, box(1.8, LENGTH - 1.8, 2.0, WIDTH - 2.0, FLOOR, DIVIDER_BOTTOM)),
        (False, box(1.1, LENGTH - 1.1, 1.3, WIDTH - 1.3, DIVIDER_BOTTOM, SEAM)),
        # Plug overmold relief leaves a continuous wall below the board.
        (False, side(*USB, 0, 0.6, 13.2, SEAM, False)),
    ]
    for far in (False, True):
        tub += [
            # A slightly wider bearing surface above the battery.
            (True, side(5, LENGTH - 5, 1.8, 2.23, 12.15, DIVIDER_BOTTOM, far)),
            (True, side(LATCH[0] - 0.4, LATCH[1] + 0.4, 1.15, 2.25, DIVIDER_BOTTOM, SEAM, far)),
            (False, side(LATCH[0] - 0.25, LATCH[1] + 0.25, 0, 1.35, 11.8, SEAM, far)),
            (False, side(LATCH[0] - 0.25, LATCH[1] + 0.25, 0, 1.76, 12.22, 13.19, far)),
            (False, side(LATCH[0] + 1.3, LATCH[1] - 1.3, 0, 0.76, 10.75, 12.05, far)),
        ]

    divider = [
        (True, box(1.4, LENGTH - 1.4, 1.6, WIDTH - 1.6, DIVIDER_BOTTOM, DIVIDER_TOP)),
        # The connector can pass edge-first with the divider lifted.
        (False, box(51.5, LENGTH - 1.4, 16, 32.5, DIVIDER_BOTTOM, DIVIDER_TOP)),
        # Low guides hold the board edges opposite the motor-to-XIAO wiring.
        (True, box(5.4, 6.1, 9.0, 18.8, DIVIDER_TOP, 14.45)),
        (True, box(10.0, 20.0, 2.95, 3.7, DIVIDER_TOP, 14.45)),
        (True, box(10.0, 19.5, 24.3, 25.1, DIVIDER_TOP, 14.45)),
        (True, box(50.15, 50.95, 5.6, 16.0, DIVIDER_TOP, 14.45)),
        (True, box(33.3, 41.0, 22.15, 22.95, DIVIDER_TOP, 14.45)),
    ]
    for far in (False, True):
        divider.append((False, side(LATCH[0] - 0.65, LATCH[1] + 0.65,
                                    1.6, 2.5, DIVIDER_BOTTOM, DIVIDER_TOP, far)))

    lid = [
        (True, box(0, LENGTH, 0, WIDTH, SEAM, HEIGHT)),
        (False, box(1.4, LENGTH - 1.4, 1.4, WIDTH - 1.4, SEAM, ROOF)),
        (False, side(*USB, 0, 1.4, SEAM, ROOF, False)),
        # Clear the board corners so the port sits close to the outside face.
        (False, side(31.8, 50.2, 0.7, 1.4, SEAM, 15.7, False)),
        # Open to the seam so the coax can be laid in with the antenna attached.
        (False, side(*ANTENNA_SLOT, 0, 1.4, SEAM, 16.4, True)),
    ]
    for far in (False, True):
        lid += [
            (False, side(LATCH[0] - 0.95, LATCH[0], 0, 1.4, SEAM, 19.0, far)),
            (False, side(LATCH[1], LATCH[1] + 0.95, 0, 1.4, SEAM, 19.0, far)),
            (False, side(*LATCH, 0, 0.2, SEAM, 18.8, far)),
            (True, side(*LATCH, 0.2, 1.16, 12.0, SEAM, far)),
            (True, side(*LATCH, 1.16, 1.65, 12.35, 13.0, far)),
        ]
        for x0, x1 in DIVIDER_STOPS:
            lid.append((True, side(x0, x1, 0.8, 2.35, SEAM, 15.4, far)))
            lid.append((True, side(x0, x1, 1.47, 2.35, 13.9, SEAM, far)))
    return {"Battery tub": tub, "Electronics divider": divider, "Snap lid": lid}


def axis_grid(recipe, axis, extent=None):
    stops = {round(bounds[index], 5) for _, bounds in recipe for index in (axis * 2, axis * 2 + 1)}
    if extent is not None:
        for step in range(9):
            value = CORNER_RADIUS * step / 8
            stops.update((round(value, 5), round(extent - value, 5)))
    elif min(stops) == 0:
        stops.update((0.25, 0.5, 0.85, 1.2))
    elif max(stops) == HEIGHT:
        stops.update(round(HEIGHT - value, 5) for value in (0.25, 0.5, 0.85, 1.2))
    return sorted(stops)


def shape_point(x, y, z, part):
    # Map the four corner squares onto circular outer corners. The same mapping
    # applies to every mating surface, which preserves the assembly clearances.
    r = CORNER_RADIUS
    cx = min(max(x, r), LENGTH - r)
    cy = min(max(y, r), WIDTH - r)
    dx, dy = x - cx, y - cy
    x = cx + dx * sqrt(1 - 0.5 * (dy / r) ** 2)
    y = cy + dy * sqrt(1 - 0.5 * (dx / r) ** 2)

    depth = z if part == "Battery tub" else HEIGHT - z if part == "Snap lid" else 2
    setbacks = {0: 0.72, 0.25: 0.52, 0.5: 0.32, 0.85: 0.12}
    inset = setbacks.get(round(depth, 5), 0)
    x = inset + x * (1 - 2 * inset / LENGTH)
    y = inset + y * (1 - 2 * inset / WIDTH)

    if part == "Snap lid":
        # Lead-in ramps on the latch hooks and printable slopes on the stops.
        ramps = (
            (12.35, (LATCH,), 1.16, 1.65, 1.20),
            (15.4, DIVIDER_STOPS, 1.4, 2.35, 1.49),
        )
        for height, spans, start, depth, tip in ramps:
            if z == height and any(x0 <= x <= x1 for x0, x1 in spans):
                near = min(y, WIDTH - y)
                if start - 0.00001 <= near <= depth + 0.00001:
                    offset = start + (near - start) * (tip - start) / (depth - start)
                    y = offset if y <= WIDTH / 2 else WIDTH - offset
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
        x, y = ((8, 8), (8, 53), (74, 8))[i - 1]
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
    write_plate(destination / "wristband_case.3mf", meshes)


if __name__ == "__main__":
    main()
