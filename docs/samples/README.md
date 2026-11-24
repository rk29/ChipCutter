# ChipCutter Sample Projects Gallery

Browse every built-in sample project included in **ChipCutter**. Click any thumbnail or title to open its full-resolution showcase page with geometry/toolpath explanations and—for multi-setup and multi-operation samples—progressive simulation screenshots captured at the exact moment **each Setup and Operation completes**.

> [!TIP]
> You can launch any sample directly from the desktop **Welcome / Sample Projects Gallery** (`File -> Sample Projects Gallery...`) or from your terminal with:
> ```bash
> chip-gui --sample <slug>
> ```
> Inspect any operation's live 3D cutting-area or rotary boundary preview on the Operation Inspector's **`[Geometry]`** tab, and configure simulation engine (`Heightmap` / `Tri-Dexel`), resolution, compute backend (`Auto` / `GPU` / `CPU`), and smooth **Mesh Skinning** under **`Settings → Simulation`** (`Ctrl+,` or the Simulation Timeline status chip).

---

## 3D STEP Solids

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![CAM Torture Block](../images/samples/cam-torture/thumb.png)](cam-torture.md) | [**CAM Torture Block**](cam-torture.md)<br>Full-featured 3D STEP benchmark exercising Facing, Pockets + Islands, Open Shoulder, Helical Counterbore, 4x Peck Drills, Outer Profile + Tabs, and 45° Edge Chamfer. | 1 Setup · 8 Ops | `--sample cam-torture` |
| [![Threaded Manifold Block](../images/samples/thread-mill/thumb.png)](thread-mill.md) | [**Threaded Manifold Block (Helical Thread Milling)**](thread-mill.md)<br>`80 × 60 × 20 mm` aluminum manifold showcasing Facing, boss deck clearing, helical pilot boring, internal `M8 × 1.25` & `1/4-20 UNC` threads, and external `M12 × 1.75` boss threading with Tri-Dexel undercut verification. | 1 Setup · 6 Ops | `--sample thread-mill` |
| [![2D Adaptive + Rest](../images/samples/adaptive-rest/thumb.png)](adaptive-rest.md) | [**2D Adaptive Clearing + Rest Machining**](adaptive-rest.md)<br>High-efficiency constant-engagement Ø10 mm trochoidal Adaptive roughing followed by Ø3 mm Rest Machining corner cleanup and Ø6 mm air-entry shoulder clearing. | 1 Setup · 5 Ops | `--sample adaptive-rest` |
| [![Feed-Optimized Adaptive Pocket](../images/samples/feed-opt-pocket/thumb.png)](feed-opt-pocket.md) | [**Feed-Optimized Adaptive Pocket (Engagement Telemetry)**](feed-opt-pocket.md)<br>Sharp-corner island pocket with Engagement-Aware Feed Optimization (corner lookahead deceleration, internal-arc centerline scaling, radial chip-thinning boost, and timeline telemetry). | 1 Setup · 2 Ops | `--sample feed-opt-pocket` |
| [![Prismatic Multi-Feature Bracket](../images/samples/multi-feature/thumb.png)](multi-feature.md) | [**Prismatic Multi-Feature Bracket**](multi-feature.md)<br>Clean starter 3D STEP bracket demonstrating planar face pocketing and 4x Ø6 mm mounting hole drilling. | 1 Setup · 2 Ops | `--sample multi-feature` |
| [![30°-Tilted WCS Alignment Bracket](../images/samples/tilted-bracket/thumb.png)](tilted-bracket.md) | [**30°-Tilted WCS Alignment Bracket**](tilted-bracket.md)<br>Unseeded STEP bracket tilted `30° X / 25° Z` for practicing SE(3) Setup WCS face/edge alignment and tight 27-point stock framing. | 1 Setup · Sandbox | `--sample tilted-bracket` |
| [![Associative CAD Revision v1](../images/samples/revision-v1/thumb.png)](revision-v1.md) | [**Associative CAD Revision Reload (v1)**](revision-v1.md)<br>Baseline Revision v1 STEP bracket seeded with 6 operations (Facing, Blind Pocket, Helical Bore, 2x Mounting Holes, Front Chamfer) prior to CAD revision reload. | 1 Setup · 6 Ops | `--sample revision-v1` |
| [![Associative CAD Revision v2](../images/samples/revision-v2/thumb.png)](revision-v2.md) | [**Associative CAD Revision Reload (v2)**](revision-v2.md)<br>Revision v2 reload showing fuzzy `GeometricSignature` re-binding of resized/shifted features plus 1-click `NeedsRepick` orphan detection on the deleted hole. | 1 Setup · 6 Ops (1 Repick) | `--sample revision-v2` |
| [![CNC Mounting Plate](../images/samples/sample-plate/thumb.png)](sample-plate.md) | [**CNC Mounting Plate (Blank STEP Sandbox)**](sample-plate.md)<br>Clean unseeded machined STEP mounting plate for practicing interactive 3D face, closed loop, and cylindrical hole picking from scratch. | 1 Setup · Sandbox | `--sample sample-plate` |

---

## Workholding & Multi-Setup Stock Carryover

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![Two-Sided Vise & Soft-Jaw Bracket](../images/samples/two-sided-vise/thumb.png)](two-sided-vise.md) | [**Two-Sided Vise & Soft-Jaw Bracket (`G54` -> `G55`)**](two-sided-vise.md)<br>Two-sided aluminum bracket showcasing Setup 1 vise + parallels + toe clamps with automatic rapid-lift avoidance and Setup 2 `180°` flip Tri-Dexel stock carryover in stepped soft jaws. | 2 Setups · 4 Ops | `--sample two-sided-vise` |
| [![Flip-Jig Dowel Pins & Auto Soft-Jaw Setup](../images/samples/flip-jig-softjaws/thumb.png)](flip-jig-softjaws.md) | [**Flip-Jig Dowel Pins & Auto Soft-Jaw Setup (`G54`–`G56`)**](flip-jig-softjaws.md)<br>Contour plate on an MDF Spoilboard with `TwoOnAxis` Flip-Jig registration pins (+5 mm poka-yoke), `180°` flipped Setup 2 carryover with `CylinderKeepOut` pins, and Setup 3 generated Auto Soft-Jaw machining. | 3 Setups · 8 Ops | `--sample flip-jig-softjaws` |

---

## Undercuts & 3D Edge Deburring

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![Undercut & 3D Deburr Block](../images/samples/undercut-deburr/thumb.png)](undercut-deburr.md) | [**Undercut & 3D Deburr Block (T-Slot, Dovetail, Lollipop)**](undercut-deburr.md)<br>`100 × 80 × 28 mm` aluminum block exercising Facing, Pre-Slot Throats, DIN 650 M8 T-Slot, `60°` Dovetail guideway, Lollipop 3D overhang undercut, and 3D Edge Deburr & Back-Chamfer in Tri-Dexel. | 1 Setup · 6 Ops | `--sample undercut-deburr` |
| [![3D Curved Arch & Underside Deburring](../images/samples/curved-deburr-3d/thumb.png)](curved-deburr-3d.md) | [**3D Curved Arch & Underside Deburring (3 Cutters)**](curved-deburr-3d.md)<br>Non-planar curved arch & overhang part exercising `Deburr3D` across ChamferMill (`FlatBvh` inside-corner wall trimming), BallEndmill curved arch rims, and Lollipop underside back-chamfering. | 1 Setup · 6 Ops | `--sample curved-deburr-3d` |
| [![External 60° Dovetail Rail & Blind Keyway](../images/samples/dovetail-rail-keyway/thumb.png)](dovetail-rail-keyway.md) | [**External 60° Dovetail Rail & Blind Entry-Bore Keyway**](dovetail-rail-keyway.md)<br>`90 × 50 × 22 mm` slide block sequencing Facing, `Ø8 mm` Entry Bore, blind Keyway via `EntryBoreCenter`, and dual-flank External `60°` Dovetail Rail milling in Tri-Dexel mode. | 1 Setup · 4 Ops | `--sample dovetail-rail-keyway` |

---

## 3D Meshes (`STL` / `OBJ` / `3MF`)

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![Sculpted Dome STL](../images/samples/mesh-dome/thumb.png)](mesh-dome.md) | [**Sculpted Dome (Watertight STL Mesh)**](mesh-dome.md)<br>R25 mm hemisphere seated on a `70 × 70 × 8 mm` pedestal machined with 3D Adaptive Roughing, 3D Waterline Semi-Finish, and Ø3 mm 3D Pencil Fillet Cleanup. | 1 Setup · 3 Ops | `--sample mesh-dome` |
| [![Relief Wave OBJ](../images/samples/mesh-wave/thumb.png)](mesh-wave.md) | [**Relief Wave (OBJ Mesh Finishing)**](mesh-wave.md)<br>`80 × 60 × 15 mm` sculpted sinusoidal relief showcasing 3D Adaptive Roughing, 45° 3D Parallel Raster Finishing, and 3D Constant-Scallop Finishing. | 1 Setup · 3 Ops | `--sample mesh-wave` |
| [![Sculpted Mount Bracket 3MF](../images/samples/mesh-3mf-bracket/thumb.png)](mesh-3mf-bracket.md) | [**Sculpted Mount Bracket (Unit-Aware 3MF Mesh)**](mesh-3mf-bracket.md)<br>`80 × 56 × 20 mm` watertight millimeter `.3mf` sculpted mounting bracket machined with 3D Adaptive Roughing and 3D Constant-Scallop Finishing. | 1 Setup · 2 Ops | `--sample mesh-3mf-bracket` |

---

## 4th-Axis Rotary (`A`-Axis)

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![Rotary Indexed Shaft](../images/samples/rotary-shaft/thumb.png)](rotary-shaft.md) | [**Rotary Indexed Shaft (4th-Axis A)**](rotary-shaft.md)<br>`Ø40 × 80 mm` shaft combining continuous `Z+X+A` rotary spiral surfacing over a collar & 3-lobe cam with 4-sided Indexed `3+1` wrenching flats (`0°/90°/180°/270°`). | 1 Setup · 5 Ops | `--sample rotary-shaft` |
| [![Wrapped Rotary Engraving](../images/samples/rotary-wrap/thumb.png)](rotary-wrap.md) | [**Wrapped Rotary Engraving (`Z+X+A`)**](rotary-wrap.md)<br>Wraps bold TrueType `"CHIP"` V-Carving and a rounded circumferential flute onto a `Ø40 × 80 mm` cylindrical blank for `G93` 4-axis or swapped-axis GRBL rotaries. | 1 Setup · 2 Ops | `--sample rotary-wrap` |
| [![Simultaneous 4-Axis Helical Flute](../images/samples/rotary-4axis/thumb.png)](rotary-4axis.md) | [**Simultaneous 4-Axis Helical Flute (`X+Y+Z+A`)**](rotary-4axis.md)<br>`Ø40 × 80 mm` 3-start helical flute shaft machined with simultaneous `X+Y+Z+A` off-centerline auto-tilted roughing and finishing (`G93` inverse-time feed). | 1 Setup · 2 Ops | `--sample rotary-4axis` |

---

## Pure-2D & DXF / SVG Vector Jobs

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![Pure-2D Router Front Panel](../images/samples/pure-2d/thumb.png)](pure-2d.md) | [**Pure-2D CNC Router Front Panel (`.dxf`)**](pure-2d.md)<br>Zero-STEP sheet-stock electronics faceplate imported directly from AutoCAD DXF with mounting holes, bossed display pocket, vent slot cutout, and perimeter tabs. | 1 Setup · 4 Ops | `--sample pure-2d` |
| [![Pure-2D SVG Badge Sign](../images/samples/svg-badge/thumb.png)](svg-badge.md) | [**Pure-2D SVG Badge Sign (`96 DPI` Bézier Biarcs)**](svg-badge.md)<br>Zero-STEP badge imported directly from `96 DPI` cubic Bézier `.svg` with nested counter-islands, annular Pocket, V-Carve accents, and outer Contour + Tabs emitting true `G2`/`G3` arcs. | 1 Setup · 3 Ops | `--sample svg-badge` |

---

## V-Carve & CAM Typography

| Preview | Sample & Overview | Setups / Ops | Launch Flag |
|---|---|---|---|
| [![V-Carve Sign & Male Inlay](../images/samples/vcarve/thumb.png)](vcarve.md) | [**V-Carve Sign & Mated Male V-Inlay**](vcarve.md)<br>Two-setup hardwood plaque pairing Setup 1 (`G54`) two-stage Flat + 60° V-Bit sign carving and perimeter cutout with Setup 2 (`G55`) mirrored Male V-Inlay plug generation. | 2 Setups · 4 Ops | `--sample vcarve` |
| [![Engraved Nameplate](../images/samples/nameplate/thumb.png)](nameplate.md) | [**Engraved Nameplate (TrueType + Hershey)**](nameplate.md)<br>Parametric R10 filleted plate featuring a 6x Flange Bolt Circle, Ø22 mm gauge window, TrueType outline `"CHIP"` pocketing, 1-pass Hershey serial engraving, and outer tabs. | 1 Setup · 5 Ops | `--sample nameplate` |

---

## Regenerating Gallery Screenshots

Every full-resolution screenshot, `480 px` thumbnail, and per-operation completion capture under `docs/images/samples/` is generated deterministically from the built-in sample loaders:

```bash
./scripts/capture_sample_gallery.sh
```

See also the step-by-step [**ChipCutter Tutorials (`00`–`22`)**](../tutorials/README.md) if you want to build each strategy from scratch.
