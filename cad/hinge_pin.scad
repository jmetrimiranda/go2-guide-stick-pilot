// Printable hinge pin + snap clip for the guide-stick clevis (replaces the M6 bolt).
// Clevis: 2 ears of 9 mm + 16 mm gap = 34 mm grip; holes are 6.4 mm (printed ~6.1-6.3).
// Pin prints lying flat (flat underside), so layers run along the pin = strong in bending/shear.
$fn = 96;
part   = "plate";   // "pin" | "clip" | "plate"
d_pin  = 5.8;       // passes the 6.4 mm holes with margin
grip   = 34.6;      // 9 + 16 + 9 + 0.6 clearance
head_d = 12;  head_t = 3;
g_w    = 2.3;       // groove width (clip is 2.0 thick)
g_d    = 4.4;       // groove core diameter
tip    = 4.5;       // after the groove; has a 1.8 mm cross hole for a wire/paperclip backup
flat   = 2.45;      // bed flat, distance below the pin axis

module pin() {
  translate([0,0,flat]) difference() {
    rotate([0,90,0]) union() {
      translate([0,0,-head_t]) cylinder(d=head_d, h=head_t);
      cylinder(d=d_pin, h=grip);
      translate([0,0,grip]) cylinder(d=g_d, h=g_w);
      translate([0,0,grip+g_w]) cylinder(d=d_pin, h=tip-0.8);
      translate([0,0,grip+g_w+tip-0.8]) cylinder(d1=d_pin, d2=d_pin-1.4, h=0.8);
    }
    translate([-10,-10,-flat-20]) cube([80,20,20]);                       // flat underside
    translate([grip+g_w+2.2,0,-10]) cylinder(d=1.8, h=20, $fn=32);       // backup cross hole
  }
}

module clip() {   // C clip, snaps into the groove; lead-in opening
  t = 2.0; di = 4.5; do = 11.5; gap = 3.4;
  difference() {
    cylinder(d=do, h=t);
    translate([0,0,-1]) cylinder(d=di, h=t+2);
    // opening with a V lead-in towards the outside
    // straight neck (narrowest = gap) from the bore out to ri+0.8, then a V lead-in
    translate([0,0,-1]) linear_extrude(t+2)
      polygon([[-gap/2,0],[gap/2,0],[gap/2,di/2+0.8],[3.8,do/2+1],[-3.8,do/2+1],[-gap/2,di/2+0.8]]);
  }
}

if (part == "pin")  pin();
if (part == "clip") clip();
if (part == "plate") {
  pin();
  translate([0,16,0]) pin();
  for (i=[0:2]) translate([10+i*14, 34, 0]) clip();
}
