# SPDX-License-Identifier: Apache-2.0
#
# generate_enclosure.py — Parametric IP54 snap-fit enclosure for SenseEdge
# predictive-maintenance sensor node.
#
# Usage:
#   freecadcmd generate_enclosure.py        # headless, exports STEP + STL
#   FreeCAD GUI -> Macro -> Run this file    # interactive
#   python generate_enclosure.py             # dry-run without FreeCAD
#
# The script creates two solid bodies (bottom shell + top lid) with:
#   - Rounded corners, mounting ears, PCB standoffs, snap-fit clips,
#     cable-gland hole, LED light-pipe hole, and ventilation slots.

from __future__ import annotations

import math
import os
import sys

# ---------------------------------------------------------------------------
# Enclosure parameters (all dimensions in millimetres)
# ---------------------------------------------------------------------------

OUTER_LENGTH = 55.0          # X dimension
OUTER_WIDTH = 45.0           # Y dimension
OUTER_HEIGHT = 25.0          # Z dimension (total: bottom + lid)
WALL_THICKNESS = 2.0         # shell wall / floor / ceiling thickness
CORNER_RADIUS = 3.0          # fillet radius on vertical edges

# Shell split
BOTTOM_HEIGHT = 17.0         # Z height of bottom shell
LID_HEIGHT = 8.0             # Z height of top lid (BOTTOM + LID == OUTER_HEIGHT)

# Mounting ears (M3)
MOUNTING_EAR_EXTENSION = 8.0        # how far ear protrudes from body
MOUNTING_EAR_WIDTH = 10.0           # ear width along the long side
MOUNTING_EAR_THICKNESS = 3.0        # ear Z thickness (same as floor)
MOUNTING_HOLE_DIA = 3.2             # M3 clearance hole
NUM_EARS_PER_SIDE = 2               # ears on each long (X) side

# PCB standoffs (M2.5)
PCB_LENGTH = 45.0                   # board X
PCB_WIDTH = 35.0                    # board Y
STANDOFF_HEIGHT = 5.0               # standoff Z above inner floor
STANDOFF_OUTER_DIA = 5.0            # standoff cylinder OD
STANDOFF_HOLE_DIA = 2.5             # M2.5 pilot / tap hole

# Cable gland (M12)
CABLE_GLAND_DIA = 12.0              # hole on short side (Y-Z plane)
CABLE_GLAND_Z_CENTRE = 10.0         # centre height from bottom outer face

# LED light pipe
LED_HOLE_DIA = 3.0                  # through-hole in lid top face
LED_OFFSET_FROM_EDGE = 5.0          # distance from nearest lid edge (X & Y)

# Snap-fit clips
CLIP_WIDTH = 6.0                    # width of each clip tab
CLIP_HEIGHT = 3.0                   # vertical hook height
CLIP_DEPTH = 1.0                    # how far the hook protrudes inward
CLIP_THICKNESS = 1.2                # material thickness of the tab
NUM_CLIPS_PER_LONG_SIDE = 2         # clips on each long (X) wall

# Ventilation slots (on each short side of bottom shell)
VENT_SLOT_WIDTH = 1.0               # slot width (Y direction on short wall)
VENT_SLOT_LENGTH = 10.0             # slot height (Z direction)
VENT_SLOT_COUNT = 3                 # number of slots per short side
VENT_SLOT_SPACING = 3.0             # centre-to-centre Y spacing
VENT_SLOT_Z_BOTTOM = 5.0            # bottom of lowest slot from outer base

# Lip / sealing rim (adds IP54 dust ingress resistance)
LIP_HEIGHT = 2.0                    # rim that the lid sits over
LIP_THICKNESS = 1.0                 # rim wall thickness

# Export directory (same folder as this script)
EXPORT_DIR = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------------------
# FreeCAD import gate
# ---------------------------------------------------------------------------

FREECAD_AVAILABLE = False

try:
    import FreeCAD  # type: ignore[import-untyped]
    import Part  # type: ignore[import-untyped]
    from FreeCAD import Base  # type: ignore[import-untyped]

    FREECAD_AVAILABLE = True
except ImportError:
    pass


# ---------------------------------------------------------------------------
# Helper geometry functions
# ---------------------------------------------------------------------------

def _rounded_rect_wire(length: float, width: float, radius: float,
                       z: float = 0.0) -> "Part.Wire":
    """Return a closed wire for a rounded rectangle centred on the XY origin
    at height *z*.  *length* is along X, *width* along Y."""
    r = min(radius, length / 2, width / 2)
    hl, hw = length / 2, width / 2

    edges: list[Part.Edge] = []

    # Corner centres (CCW from bottom-right)
    corners = [
        (hl - r, -hw + r),
        (hl - r, hw - r),
        (-hl + r, hw - r),
        (-hl + r, -hw + r),
    ]

    # Straight segments between arcs (bottom, right, top, left)
    straights = [
        ((-hl + r, -hw, z), (hl - r, -hw, z)),   # bottom edge
        ((hl, -hw + r, z), (hl, hw - r, z)),      # right edge
        ((hl - r, hw, z), (-hl + r, hw, z)),       # top edge
        ((-hl, hw - r, z), (-hl, -hw + r, z)),     # left edge
    ]

    start_angles = [270, 0, 90, 180]

    for i in range(4):
        # Straight segment
        p1 = Base.Vector(*straights[i][0])
        p2 = Base.Vector(*straights[i][1])
        edges.append(Part.makeLine(p1, p2))

        # Arc
        cx, cy = corners[i]
        centre = Base.Vector(cx, cy, z)
        sa = start_angles[i]
        arc = Part.makeCircle(r, centre, Base.Vector(0, 0, 1), sa, sa + 90)
        edges.append(Part.Edge(arc))

    wire = Part.Wire(edges)
    return wire


def _rounded_rect_face(length: float, width: float, radius: float,
                       z: float = 0.0) -> "Part.Face":
    """Convenience: face from rounded rectangle wire."""
    return Part.Face(_rounded_rect_wire(length, width, radius, z))


def _rounded_box(length: float, width: float, height: float,
                 radius: float, z_base: float = 0.0) -> "Part.Shape":
    """Extruded rounded-rectangle box."""
    face = _rounded_rect_face(length, width, radius, z_base)
    return face.extrude(Base.Vector(0, 0, height))


def _cylinder(x: float, y: float, z: float, dia: float,
              height: float) -> "Part.Shape":
    """Upward cylinder at (x, y, z)."""
    return Part.makeCylinder(dia / 2, height, Base.Vector(x, y, z))


def _cylinder_hole(x: float, y: float, z: float, dia: float,
                   depth: float) -> "Part.Shape":
    """Tool cylinder (for subtraction)."""
    return _cylinder(x, y, z, dia, depth)


# ---------------------------------------------------------------------------
# Bottom shell builder
# ---------------------------------------------------------------------------

def build_bottom_shell() -> "Part.Shape":
    """Return the solid shape for the bottom half of the enclosure."""

    inner_length = OUTER_LENGTH - 2 * WALL_THICKNESS
    inner_width = OUTER_WIDTH - 2 * WALL_THICKNESS
    inner_radius = max(CORNER_RADIUS - WALL_THICKNESS, 0.5)

    # 1. Outer solid
    outer = _rounded_box(OUTER_LENGTH, OUTER_WIDTH, BOTTOM_HEIGHT,
                         CORNER_RADIUS)

    # 2. Inner cavity (hollow out from top, leave floor)
    cavity = _rounded_box(inner_length, inner_width,
                          BOTTOM_HEIGHT - WALL_THICKNESS,
                          inner_radius, WALL_THICKNESS)
    shell = outer.cut(cavity)

    # 3. Sealing lip (rim around inner top edge for lid to sit into)
    lip_outer = _rounded_box(inner_length, inner_width, LIP_HEIGHT,
                             inner_radius, BOTTOM_HEIGHT)
    lip_inner_l = inner_length - 2 * LIP_THICKNESS
    lip_inner_w = inner_width - 2 * LIP_THICKNESS
    lip_inner_r = max(inner_radius - LIP_THICKNESS, 0.3)
    lip_cavity = _rounded_box(lip_inner_l, lip_inner_w, LIP_HEIGHT,
                              lip_inner_r, BOTTOM_HEIGHT)
    lip = lip_outer.cut(lip_cavity)
    shell = shell.fuse(lip)

    # 4. Mounting ears (2 per long side, on the +Y and -Y faces)
    ear_z = 0.0
    ear_positions_x = _distribute(NUM_EARS_PER_SIDE, OUTER_LENGTH,
                                  MOUNTING_EAR_WIDTH)

    for side_sign in (1, -1):  # +Y and -Y
        for ex in ear_positions_x:
            # Ear centre in Y just outside the body
            ey = side_sign * (OUTER_WIDTH / 2 + MOUNTING_EAR_EXTENSION / 2)
            ear_box = Part.makeBox(
                MOUNTING_EAR_WIDTH, MOUNTING_EAR_EXTENSION,
                MOUNTING_EAR_THICKNESS,
                Base.Vector(ex - MOUNTING_EAR_WIDTH / 2,
                            ey - MOUNTING_EAR_EXTENSION / 2,
                            ear_z))
            shell = shell.fuse(ear_box)

            # Through-hole
            hole_centre_y = side_sign * (OUTER_WIDTH / 2
                                         + MOUNTING_EAR_EXTENSION / 2)
            hole = _cylinder_hole(ex, hole_centre_y, ear_z,
                                  MOUNTING_HOLE_DIA,
                                  MOUNTING_EAR_THICKNESS)
            shell = shell.cut(hole)

    # 5. PCB standoffs
    pcb_offset_x = PCB_LENGTH / 2 - 2.5   # 2.5 mm inboard from board edge
    pcb_offset_y = PCB_WIDTH / 2 - 2.5
    standoff_positions = [
        (pcb_offset_x, pcb_offset_y),
        (pcb_offset_x, -pcb_offset_y),
        (-pcb_offset_x, pcb_offset_y),
        (-pcb_offset_x, -pcb_offset_y),
    ]

    for sx, sy in standoff_positions:
        post = _cylinder(sx, sy, WALL_THICKNESS, STANDOFF_OUTER_DIA,
                         STANDOFF_HEIGHT)
        shell = shell.fuse(post)
        tap_hole = _cylinder_hole(sx, sy, WALL_THICKNESS, STANDOFF_HOLE_DIA,
                                  STANDOFF_HEIGHT)
        shell = shell.cut(tap_hole)

    # 6. Cable gland hole (centred on the -X short face)
    gland_x = -OUTER_LENGTH / 2
    gland_y = 0.0
    gland_z = CABLE_GLAND_Z_CENTRE
    gland_cyl = Part.makeCylinder(
        CABLE_GLAND_DIA / 2, WALL_THICKNESS * 3,
        Base.Vector(gland_x - WALL_THICKNESS, gland_y, gland_z),
        Base.Vector(1, 0, 0))
    shell = shell.cut(gland_cyl)

    # 7. Ventilation slots on each short side (-X and +X faces)
    for x_sign in (-1, 1):
        slot_x = x_sign * OUTER_LENGTH / 2
        total_span = (VENT_SLOT_COUNT - 1) * VENT_SLOT_SPACING
        for i in range(VENT_SLOT_COUNT):
            sy = -total_span / 2 + i * VENT_SLOT_SPACING
            sz = VENT_SLOT_Z_BOTTOM
            # Slot as a thin box punched through the wall
            slot = Part.makeBox(
                WALL_THICKNESS * 3, VENT_SLOT_WIDTH, VENT_SLOT_LENGTH,
                Base.Vector(slot_x - WALL_THICKNESS * 1.5,
                            sy - VENT_SLOT_WIDTH / 2, sz))
            shell = shell.cut(slot)

    # 8. Snap-fit clip tabs (protruding hooks on outside of long walls)
    clip_positions_x = _distribute(NUM_CLIPS_PER_LONG_SIDE, OUTER_LENGTH,
                                   CLIP_WIDTH)
    for side_sign in (1, -1):
        for cx in clip_positions_x:
            cy_base = side_sign * (OUTER_WIDTH / 2 - WALL_THICKNESS)
            # Tab body (thin vertical strip on inner wall)
            tab = Part.makeBox(
                CLIP_WIDTH, CLIP_THICKNESS, CLIP_HEIGHT + 2,
                Base.Vector(cx - CLIP_WIDTH / 2,
                            cy_base - (CLIP_THICKNESS / 2 if side_sign > 0
                                       else CLIP_THICKNESS / 2),
                            BOTTOM_HEIGHT - CLIP_HEIGHT - 2))
            # Hook nub at top of tab (protrudes inward)
            hook = Part.makeBox(
                CLIP_WIDTH, CLIP_DEPTH, CLIP_HEIGHT,
                Base.Vector(cx - CLIP_WIDTH / 2,
                            (cy_base - CLIP_DEPTH if side_sign > 0
                             else cy_base),
                            BOTTOM_HEIGHT - CLIP_HEIGHT))
            clip = tab.fuse(hook)
            shell = shell.fuse(clip)

    return shell


# ---------------------------------------------------------------------------
# Top lid builder
# ---------------------------------------------------------------------------

def build_lid() -> "Part.Shape":
    """Return the solid shape for the top lid."""

    inner_length = OUTER_LENGTH - 2 * WALL_THICKNESS
    inner_width = OUTER_WIDTH - 2 * WALL_THICKNESS
    inner_radius = max(CORNER_RADIUS - WALL_THICKNESS, 0.5)

    z_base = BOTTOM_HEIGHT  # lid sits on top of bottom shell

    # 1. Outer solid
    outer = _rounded_box(OUTER_LENGTH, OUTER_WIDTH, LID_HEIGHT,
                         CORNER_RADIUS, z_base)

    # 2. Inner cavity (hollow from bottom, leave ceiling)
    cavity_height = LID_HEIGHT - WALL_THICKNESS
    cavity = _rounded_box(inner_length, inner_width, cavity_height,
                          inner_radius, z_base)
    lid = outer.cut(cavity)

    # 3. Recess for sealing lip — a groove that accepts the bottom shell lip
    lip_recess_l = inner_length - 2 * LIP_THICKNESS + 0.3  # 0.3 clearance
    lip_recess_w = inner_width - 2 * LIP_THICKNESS + 0.3
    lip_recess_r = max(inner_radius - LIP_THICKNESS, 0.3)
    recess_outer = _rounded_box(inner_length + 0.3, inner_width + 0.3,
                                LIP_HEIGHT, inner_radius, z_base)
    recess_inner = _rounded_box(lip_recess_l, lip_recess_w, LIP_HEIGHT,
                                lip_recess_r, z_base)
    recess = recess_outer.cut(recess_inner)
    # Cut this groove into the lid interior so the lip slots in
    lid = lid.cut(recess)

    # 4. LED light-pipe hole (from top face, near one corner)
    led_x = OUTER_LENGTH / 2 - LED_OFFSET_FROM_EDGE
    led_y = OUTER_WIDTH / 2 - LED_OFFSET_FROM_EDGE
    led_z = z_base + LID_HEIGHT - WALL_THICKNESS
    led_hole = _cylinder_hole(led_x, led_y, led_z, LED_HOLE_DIA,
                              WALL_THICKNESS * 2)
    lid = lid.cut(led_hole)

    # 5. Snap-fit clip slots (rectangular cut-outs on inner long walls)
    clip_positions_x = _distribute(NUM_CLIPS_PER_LONG_SIDE, OUTER_LENGTH,
                                   CLIP_WIDTH)
    clearance = 0.3  # printing clearance
    for side_sign in (1, -1):
        for cx in clip_positions_x:
            slot_y_base = side_sign * (OUTER_WIDTH / 2 - WALL_THICKNESS)
            slot = Part.makeBox(
                CLIP_WIDTH + clearance,
                CLIP_DEPTH + CLIP_THICKNESS + clearance,
                CLIP_HEIGHT + clearance,
                Base.Vector(
                    cx - (CLIP_WIDTH + clearance) / 2,
                    slot_y_base - ((CLIP_DEPTH + CLIP_THICKNESS + clearance) / 2
                                   if side_sign > 0
                                   else (CLIP_DEPTH + CLIP_THICKNESS
                                         + clearance) / 2),
                    z_base))
            lid = lid.cut(slot)

    return lid


# ---------------------------------------------------------------------------
# Utility
# ---------------------------------------------------------------------------

def _distribute(count: int, total_length: float,
                item_width: float) -> list[float]:
    """Return *count* evenly-spaced X positions centred on 0, with items of
    *item_width* fitting comfortably inside *total_length*."""
    if count == 1:
        return [0.0]
    usable = total_length - 2 * WALL_THICKNESS - item_width
    spacing = usable / (count - 1)
    start = -usable / 2
    return [start + i * spacing for i in range(count)]


# ---------------------------------------------------------------------------
# Export helpers
# ---------------------------------------------------------------------------

def export_shape(shape: "Part.Shape", name: str, directory: str) -> None:
    """Write *shape* to STEP and STL files in *directory*."""
    step_path = os.path.join(directory, f"{name}.step")
    stl_path = os.path.join(directory, f"{name}.stl")

    shape.exportStep(step_path)
    print(f"  STEP -> {step_path}")

    mesh = FreeCAD.ActiveDocument.addObject("Mesh::Feature", f"{name}_mesh")
    mesh.Mesh = MeshFromShape(shape)
    import Mesh  # type: ignore[import-untyped]
    Mesh.export([mesh], stl_path)
    print(f"  STL  -> {stl_path}")


def export_shape_simple(shape: "Part.Shape", name: str,
                        directory: str) -> None:
    """Export STEP + STL without requiring an active FreeCAD document for the
    STL (uses the shape tessellation directly)."""
    step_path = os.path.join(directory, f"{name}.step")
    stl_path = os.path.join(directory, f"{name}.stl")

    # STEP export
    shape.exportStep(step_path)
    print(f"  STEP -> {step_path}")

    # STL export via mesh tessellation
    try:
        import Mesh  # type: ignore[import-untyped]
        import MeshPart  # type: ignore[import-untyped]
        mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=0.1,
                                      AngularDeflection=0.5)
        mesh.write(stl_path)
        print(f"  STL  -> {stl_path}")
    except Exception as exc:
        print(f"  STL export skipped ({exc}); STEP file is available.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    """Build both enclosure halves and export them."""

    if not FREECAD_AVAILABLE:
        print("=" * 60)
        print("FreeCAD is not available in this Python environment.")
        print()
        print("To generate the enclosure geometry, run this script with")
        print("the FreeCAD command-line interpreter:")
        print()
        print("    freecadcmd generate_enclosure.py")
        print()
        print("Or open it as a macro inside the FreeCAD GUI.")
        print()
        print("Install FreeCAD: https://www.freecad.org/downloads.php")
        print("=" * 60)
        print()
        print("Design summary (all dimensions in mm):")
        print(f"  Outer envelope : {OUTER_LENGTH} x {OUTER_WIDTH}"
              f" x {OUTER_HEIGHT}")
        print(f"  Bottom shell   : {OUTER_LENGTH} x {OUTER_WIDTH}"
              f" x {BOTTOM_HEIGHT}")
        print(f"  Top lid        : {OUTER_LENGTH} x {OUTER_WIDTH}"
              f" x {LID_HEIGHT}")
        print(f"  Wall thickness : {WALL_THICKNESS}")
        print(f"  Corner radius  : {CORNER_RADIUS}")
        print(f"  Mounting ears  : 4x M3 ({MOUNTING_HOLE_DIA} mm holes)")
        print(f"  PCB standoffs  : 4x M2.5, {STANDOFF_HEIGHT} mm tall")
        print(f"  Cable gland    : M12 ({CABLE_GLAND_DIA} mm hole)")
        print(f"  LED light pipe : {LED_HOLE_DIA} mm hole")
        print(f"  Snap-fit clips : {NUM_CLIPS_PER_LONG_SIDE * 2} tabs "
              f"+ matching slots")
        print(f"  Vent slots     : {VENT_SLOT_COUNT} x {VENT_SLOT_WIDTH}"
              f" x {VENT_SLOT_LENGTH} per short side")
        return

    # Ensure output directory exists
    os.makedirs(EXPORT_DIR, exist_ok=True)

    # Create a FreeCAD document (required for some export paths)
    doc = FreeCAD.newDocument("SenseEdge_Enclosure")

    print("Building bottom shell ...")
    bottom = build_bottom_shell()
    bottom_obj = doc.addObject("Part::Feature", "SenseEdge_Bottom")
    bottom_obj.Shape = bottom

    print("Building top lid ...")
    lid = build_lid()
    lid_obj = doc.addObject("Part::Feature", "SenseEdge_Lid")
    lid_obj.Shape = lid

    doc.recompute()

    print("Exporting ...")
    export_shape_simple(bottom, "senseedge_bottom", EXPORT_DIR)
    export_shape_simple(lid, "senseedge_lid", EXPORT_DIR)

    print()
    print("Done.  Files written to:", EXPORT_DIR)


if __name__ == "__main__":
    main()
