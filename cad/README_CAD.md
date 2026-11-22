# CAD files (guide-stick pilot)

Units: mm. Material: PETG. Tube: 3K carbon fibre, OD 27 / ID 25 mm, 1 m long.

## In use (final, 22 Nov 2026)

- `tower_plug.stl`: plug that replaces the sensor mast in the RollCall v21 tower (pin Ø26.8 × 60 mm, conical cap, clevis on top). It is fixed by screws in the tower's 4 lower holes (51.75 mm below the top), which land in the plug's guide dimples. The arrow on the cap points to the robot's head. The hinge axis is left-right: pitch is free, yaw is locked by the tongue in the clevis. Print standing, pin down.
- `socket_27mm.stl`: tube socket with the hinge tongue. In the built prototype the tube is fixed with 2 screws through the socket and the tube. Print standing, open end down.
- `hinge_pin.stl` + `hinge_clip.stl`: printed hinge pin (Ø5.8 mm, replaces the M6 bolt) and snap-on C clip. `hinge_pin_print_plate.stl` holds 2 pins + 3 clips, already oriented for printing (PETG, 0.12 mm layers, 4 walls, 100 % infill, 3 mm brim). Two or three layers of electrical tape on each tongue face remove the side play.
- `guide_stick.scad`, `hinge_pin.scad`: parametric sources (OpenSCAD).
- `ref_v21_mod.stl`: reference geometry of the RollCall v21 plate, used only for the interference check.
- `montagem_torre_v21.png`: assembly preview.

## Optional

- `handle_27mm.stl`: T handle. Not used; the phone is taped to the tube near the hand.

## Obsolete (kept for history, do not print)

- The strap `base` and the `sleeve` parts in `guide_stick.scad`, and every 22 mm part. They come from the first version, before the tube was measured and before the v21 tower was used.
