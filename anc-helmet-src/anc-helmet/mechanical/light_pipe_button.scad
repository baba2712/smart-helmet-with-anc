// Status light pipe + button plunger through the cup shell (main board, bottom edge).
// Print the light pipe in clear PETG (or use a 3 mm clear acrylic rod);
// the plunger in TPU so it seals against the shell.
include <params.scad>

reach = cup_depth - batt[2] - standoff_h - pcb_t - 2;   // board surface -> shell outside

module light_pipe() {
    // three LEDs sit 2 mm apart at the board edge; one 6 x 2.5 mm bar collects all three
    cube([6.5, 2.5, reach]);
    translate([-1, -1, reach - 1.2]) cube([8.5, 4.5, 1.2]);   // flange on the outside
}

module button_plunger() {
    cylinder(d = 4, h = reach);
    translate([0, 0, reach - 1.5]) cylinder(d = 8, h = 1.5);  // cap
    cylinder(d = 6, h = 1.0);                                 // retaining collar inside
}

light_pipe();
translate([16, 1.25, 0]) button_plunger();
