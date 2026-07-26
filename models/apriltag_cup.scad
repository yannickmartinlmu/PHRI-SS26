// AprilTag mug: straight cylindrical cup with tag36h11 ID0 engraved on inside floor.
// Bit pattern pulled from AprilRobotics/apriltag-imgs/tag36h11/tag36_11_00000.png (verified, not hand-guessed).

cup_od          = 80;   // outer diameter, mm
cup_height      = 100;  // mm
wall            = 3;    // straight wall thickness, mm
floor_thickness = 4;    // mm
tag_size        = 40;   // overall span of the 8x8 tag (border+data), mm
engrave_depth   = 1;    // mm, recessed into the inside floor
$fn = 128;

// tag36h11 id0, 8x8 grid = 1-module black border + 6x6 payload. 1 = black = recessed.
tag = [
  [1,1,1,1,1,1,1,1],
  [1,0,0,1,0,1,0,1],
  [1,1,0,0,0,1,0,1],
  [1,1,0,0,1,1,1,1],
  [1,0,1,0,1,1,1,1],
  [1,1,0,1,0,0,1,1],
  [1,1,1,1,0,1,1,1],
  [1,1,1,1,1,1,1,1],
];

module cup_shell() {
  difference() {
    cylinder(h = cup_height, d = cup_od);
    translate([0, 0, floor_thickness])
      cylinder(h = cup_height, d = cup_od - 2 * wall);
  }
}

module tag_recess() {
  m = tag_size / 8;
  cut_h = engrave_depth + 0.1; // small overlap past the top face, avoids z-fighting
  for (row = [0:7])
    for (col = [0:7])
      if (tag[row][col] == 1)
        translate([
          (col - 3.5) * m,
          (3.5 - row) * m,
          floor_thickness - engrave_depth / 2 + 0.05
        ])
          cube([m + 0.01, m + 0.01, cut_h], center = true);
}

difference() {
  cup_shell();
  tag_recess();
}
