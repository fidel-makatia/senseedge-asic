<!-- SPDX-License-Identifier: Apache-2.0 -->

# SenseEdge Enclosure

Parametric IP54-rated snap-fit enclosure for the SenseEdge predictive
maintenance sensor node.  The enclosure is defined entirely in
`generate_enclosure.py` and produces two parts: a **bottom shell** and a
**top lid**.

## Running the script

### With FreeCAD (generates STEP + STL files)

```bash
freecadcmd generate_enclosure.py
```

This writes four files into the `enclosure/` directory:

| File                    | Format |
|-------------------------|--------|
| `senseedge_bottom.step` | STEP   |
| `senseedge_bottom.stl`  | STL    |
| `senseedge_lid.step`    | STEP   |
| `senseedge_lid.stl`     | STL    |

You can also open FreeCAD, go to **Macro > Run Macro**, and select
`generate_enclosure.py`.  Both parts will appear in the 3D view.

### Without FreeCAD (dry-run)

```bash
python generate_enclosure.py
```

Prints a design summary and installation instructions.  Useful for
reviewing parameters without generating geometry.

### Customisation

All dimensions are defined as constants at the top of the script.  Edit
them and re-run to regenerate the enclosure.

## Recommended 3D-printing material

| Property                | ASA            | PETG           |
|-------------------------|----------------|----------------|
| Max service temperature | ~95 °C         | ~75 °C         |
| UV resistance           | Good           | Moderate       |
| Chemical resistance     | Good           | Good           |
| Layer adhesion          | Good           | Excellent      |

**ASA** is preferred for outdoor or high-temperature industrial
environments.  **PETG** is a good alternative when UV exposure is limited.

## Print settings

| Parameter        | Value                  |
|------------------|------------------------|
| Layer height     | 0.2 mm                 |
| Perimeters       | 3                      |
| Infill           | 20 % (grid or gyroid)  |
| Supports         | Not required (design is self-supporting) |
| Brim             | 5 mm recommended for ASA |
| Nozzle           | 0.4 mm                 |

Print the bottom shell **upside-down** (open face up) and the lid
**right-side up** (flat ceiling on the build plate) for best surface
finish and no supports.

## Post-processing

1. **Snap-fit clips** — Light sanding (400-grit) on the clip hooks and
   matching slots ensures a smooth, repeatable snap.
2. **Mounting ears** — Press M3 heat-set threaded inserts into the four
   ear holes using a soldering iron at ~220 °C (ASA) or ~240 °C (PETG).
3. **Cable gland** — Thread an M12 cable gland into the hole on the short
   side.  Apply a thin bead of silicone sealant around the gland for IP54
   compliance.
4. **LED light pipe** — Insert a 3 mm acrylic or silicone light pipe into
   the lid hole and secure with a drop of CA glue.
5. **Sealing** — For full IP54 dust/splash protection, place a 1 mm
   silicone gasket or apply silicone grease on the sealing lip before
   closing the lid.
