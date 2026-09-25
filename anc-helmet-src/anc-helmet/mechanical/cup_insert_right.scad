// Right cup insert: satellite board + driver baffle (same baffle as the left cup).
include <params.scad>

part = "all";   // openscad -D 'part="..."' exports one part
use <cup_insert_left.scad>

tray_t = 1.6;
gap    = 0.6;

module right_tray() {
    difference() {
        union() {
            ellipse(cup_a - 2 * gap, cup_b - 2 * gap, tray_t);
            difference() {
                ellipse(cup_a - 2 * gap, cup_b - 2 * gap, tray_t + 3);
                translate([0, 0, -1]) ellipse(cup_a - 2 * gap - 3.2, cup_b - 2 * gap - 3.2, 6);
            }
            for (h = sat_holes)
                translate([h[0] - sat_w / 2, sat_h / 2 - h[1], 0]) cylinder(d = 5.5, h = tray_t + standoff_h + 6);
        }
        for (h = sat_holes)
            translate([h[0] - sat_w / 2, sat_h / 2 - h[1], tray_t + 1]) cylinder(d = m2_insert_d, h = 20);
        translate([-cup_a / 2 + 6, 0, -1]) cylinder(d = 5, h = 10);     // outside mic cable
        translate([cup_a / 2 - 6, 0, -1]) cylinder(d = 5, h = 10);      // headband cable
    }
}

if (part == "all") right_tray();
if (part == "tray") right_tray();
if (part == "all") translate([0, cup_b + 10, 0]) baffle();
if (part == "baffle") baffle();
