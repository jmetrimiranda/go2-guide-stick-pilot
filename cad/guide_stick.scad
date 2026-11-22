// Guide-stick pilot - Go2 guide stick fixture (parametric)
// Jorge Metri Miranda, 2026. Units: mm.
// Pitch hinge is free (filters trot bounce); yaw is locked by the tongue in the clevis,
// so turning cues are transmitted to the hand.
//
// part = "tower_plug" | "socket" | "handle" | "sleeve" | "assembly" | "base" (old strap base)
part = "tower_plug";

// ---- parameters you may change ----
stick_d      = 27.0;  // diameter of the RIGID stick (measure with caliper)
thin_d       = 10.0;  // diameter of the FLEXIBLE stick (for the reducer sleeve)
clear        = 0.5;   // diametral clearance for the stick bores
bolt_d       = 6.0;   // hinge bolt (M6). M5 also works with a little play
plate_l      = 140;   // base plate length (robot forward = +x)
plate_w      = 66;    // base plate width
plate_t      = 5;
strap_w      = 25;    // velcro / strap width
ear_t        = 8;     // clevis ear thickness
gap          = 16;    // clevis gap (tongue is gap-1)
hinge_h      = 20;    // hinge axis height above plate top
ear_len      = 30;    // ear length along x
ear_x        = -22;   // ear centre along x (stick points to -x, behind the robot)
$fn = 72;

tongue_t = gap - 1.0;
tongue_w = 26;

// ---------------- BASE ----------------
module rounded_plate(l, w, t, r=6) {
  linear_extrude(t) offset(r) square([l-2*r, w-2*r], center=true);
}
module strap_slot() { cube([strap_w+3, 4.5, 3*plate_t], center=true); }

module ear(side) {
  // side = +1 / -1
  y0 = side*(gap/2 + ear_t/2);
  translate([ear_x, y0, plate_t])
    rotate([90,0,0]) linear_extrude(ear_t, center=true)
      hull() {
        translate([-ear_len/2, 0]) square([ear_len, 0.01]);
        translate([0, hinge_h]) circle(d=ear_len);
      }
  // gusset on the outer face
  yo = side*(gap/2 + ear_t);
  translate([ear_x, 0, plate_t])
    rotate([90,0,90]) linear_extrude(ear_len-4, center=true)
      polygon([[yo,0],[yo+side*10,0],[yo,12]]);
}

module base() {
  difference() {
    union() {
      rounded_plate(plate_l, plate_w, plate_t);
      ear(+1); ear(-1);
    }
    // hinge hole
    translate([ear_x, 0, plate_t+hinge_h]) rotate([90,0,0])
      cylinder(d=bolt_d+0.4, h=gap+2*ear_t+30, center=true);
    // strap slots (two straps across the body)
    for (x=[plate_l/2-20, -plate_l/2+20]) for (s=[1,-1])
      translate([x, s*(plate_w/2-9), 0]) strap_slot();
    // forward arrow
    translate([plate_l/2-48, 0, plate_t-0.8]) linear_extrude(1)
      text("FRONT >", size=6, halign="center", valign="center");
  }
}


// ---------------- TOWER PLUG (fits the Ø27.2 mast socket of the RollCall v21 plate) ----------------
// Replaces the sensor mast: slides into the v21 tower and is held by the same 4 M3 grub screws
// (45° diagonals, 51.75 mm below the tower top). Pitch hinge on top, yaw locked by the grub screws.
tower_bore   = 27.2;   // v21 socket bore (measured from the STL)
tower_od     = 34.2;   // v21 tower top outer diameter
grub_depth   = 51.75;  // grub-screw axis below the tower top
plug_len     = 60;
plug_d       = tower_bore - 0.4;   // 26.8: the grub screws centre and clamp it
cap_d        = 46;
cap_h        = 11;
pe_t         = 9;      // ear thickness on the plug
pe_len       = 30;
module tower_plug() {
  difference() {
    union() {
      // plug (goes down into the tower), with a lead-in chamfer
      translate([0,0,-plug_len]) cylinder(d1=plug_d-2, d2=plug_d, h=2);
      translate([0,0,-plug_len+2]) cylinder(d=plug_d, h=plug_len-2+0.01);
      // cap: 45-degree cone from the pin outwards (no flat overhang, prints without support);
      // it rests on the inner edge of the tower top
      cylinder(d1=plug_d, d2=cap_d, h=(cap_d-plug_d)/2);
      translate([0,0,(cap_d-plug_d)/2-0.01]) cylinder(d=cap_d, h=cap_h-(cap_d-plug_d)/2+0.01);
      // clevis ears (hinge axis along y = robot left-right)
      for (s=[1,-1]) translate([0, s*(gap/2+pe_t/2), cap_h])
        rotate([90,0,0]) linear_extrude(pe_t, center=true)
          hull() {
            translate([-pe_len/2,0]) square([pe_len, 0.01]);
            translate([0, hinge_h]) circle(d=pe_len);
          }
    }
    // hinge hole
    translate([0,0,cap_h+hinge_h]) rotate([90,0,0]) cylinder(d=bolt_d+0.4, h=60, center=true);
    // pilot dimples for the 4 grub screws
    for (a=[45,135,225,315]) rotate([0,0,a]) translate([plug_d/2-0.8,0,-grub_depth])
      rotate([0,90,0]) cylinder(d=2.6, h=5);
    // arrow on the cap: ears must be parallel to the robot body (arrow = robot forward)
    translate([cap_d/2-4.5,0,cap_h-0.8]) linear_extrude(1) polygon([[3,0],[-2.5,3],[-2.5,-3]]);
  }
}

// ---------------- SOCKET (prints standing, open bore on the bed) ----------------
module tie_groove(z, od) {
  translate([0,0,z]) difference() {
    cylinder(d=od+2, h=5);
    cylinder(d=od-3, h=5);
  }
}
module socket() {
  od = stick_d + clear + 8;
  bore_depth = 45;
  body_l = bore_depth + 5;
  difference() {
    union() {
      cylinder(d=od, h=body_l);
      hull() {
        translate([0,0,body_l-0.01]) cylinder(d=od, h=0.01);
        translate([-tongue_w/2, -tongue_t/2, body_l+8]) cube([tongue_w, tongue_t, 0.01]);
      }
      // tongue with rounded end
      translate([0,0,body_l+8]) rotate([90,0,0]) linear_extrude(tongue_t, center=true)
        hull() {
          translate([-tongue_w/2,0]) square([tongue_w, 0.01]);
          translate([0, 30-tongue_w/2]) circle(d=tongue_w);
        }
    }
    translate([0,0,-0.1]) cylinder(d=stick_d+clear, h=bore_depth+0.1);
    // clamp split + zip-tie grooves
    translate([od/2-3, -1, -0.1]) cube([6, 2, 36]);
    tie_groove(8, od); tie_groove(26, od);
    // hinge hole
    translate([0,0,body_l+8+30-tongue_w/2]) rotate([90,0,0])
      cylinder(d=bolt_d+0.4, h=tongue_t+2, center=true);
  }
}

// ---------------- HANDLE (prints flat, T shape) ----------------
module oct_prism(ff, l) { // octagon with flats on top/bottom, extruded along +x
  rotate([0,90,0]) rotate([0,0,22.5]) cylinder(d=ff/cos(22.5), h=l, $fn=8);
}
module handle() {
  ff = stick_d + clear + 10;
  tube_l = 55; grip_l = 120; grip_ff = 32;
  h = max(ff, grip_ff);
  difference() {
    union() {
      translate([0,0,h/2]) oct_prism(ff, tube_l+1);
      translate([tube_l+grip_ff/2, 0, h/2]) rotate([0,0,90])
        translate([-grip_l/2,0,0]) oct_prism(grip_ff, grip_l);
    }
    // bore along x, teardrop pointing up so it prints without support
    translate([-0.1,0,h/2]) teardrop_up(stick_d+clear, 50.1);
    // split + grooves
    translate([-0.1, -1, h/2]) cube([36, 2, h]);
    for (x=[8, 26]) translate([x,0,h/2]) difference() {
      rotate([0,90,0]) cylinder(d=ff*1.3, h=5);
      rotate([0,90,0]) rotate([0,0,22.5]) cylinder(d=(ff-3)/cos(22.5), h=5, $fn=8);
    }
    // flat bottom guarantee
    translate([-50,-100,-20]) cube([400,200,20]);
  }
}
module teardrop_up(d, l) {
  // bore along +x with a 45-degree peak toward +z
  rotate([0,90,0]) linear_extrude(l) union() {
    circle(d=d);
    rotate(135) square(d/2);         // peak along 2D -x, which maps to +z
  }
}

// ---------------- REDUCER SLEEVE (rigid bore -> flexible stick) ----------------
module sleeve() {
  difference() {
    cylinder(d=stick_d+clear-0.3, h=45);
    translate([0,0,-0.1]) cylinder(d=thin_d+0.4, h=45.2);
    translate([0,-0.75,-0.1]) cube([stick_d, 1.5, 45.2]);
  }
}

if (part=="base") base();
if (part=="tower_plug") tower_plug();
if (part=="socket") socket();
if (part=="handle") handle();
if (part=="sleeve") sleeve();
if (part=="assembly") {
  color("lightgray") translate([0,0,-119]) import("ref_v21_mod.stl");
  color("steelblue") tower_plug();
  translate([0,0,cap_h+hinge_h]) rotate([0,90+20,0])
    translate([0,0,-(50+8+30-tongue_w/2)]) color("orange") socket();
}
