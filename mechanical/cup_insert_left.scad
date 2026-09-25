// Left cup insert: holds the main board + battery against the back wall of the cup,
// and carries the 40 mm driver on a baffle in front of them (toward the ear).
//
//   back wall of cup | battery | main board (components facing the ear) | baffle + driver | foam | ear
//
// Print in PETG or ASA (PLA softens in a hot car/plant), 0.2 mm layers, 3 perimeters.
include <params.scad>

part = "all";   // openscad -D 'part="..."' exports one part

tray_t   = 1.6;
gap      = 0.6;            // clearance to the cup wall
clip_h   = 3;

module outline(t, shrink = 0) {
    ellipse(cup_a - 2 * (gap + shrink), cup_b - 2 * (gap + shrink), t);
}

// ---------------- rear tray: battery pocket + board standoffs ----------------
module rear_tray() {
    difference() {
        union() {
            outline(tray_t);
            // rim wall so the tray self-centres in the cup
            difference() { outline(clip_h + tray_t); translate([0, 0, -1]) outline(clip_h + 3, 1.6); }
            // battery fence
            translate([-batt[0] / 2 - 1.2, -batt[1] / 2 - 1.2, 0])
                difference() {
                    cube([batt[0] + 2.4, batt[1] + 2.4, tray_t + batt[2]]);
                    translate([1.2, 1.2, tray_t]) cube([batt[0], batt[1], batt[2] + 1]);
                    translate([batt[0] / 2 - 6, -1, tray_t + 3]) cube([12, 5, batt[2]]);   // lead exit
                }
            // standoffs for the main board, above the battery
            for (h = main_holes)
                translate([h[0] - main_w / 2, main_h / 2 - h[1], 0])
                    cylinder(d = 5.5, h = tray_t + batt[2] + standoff_h);
        }
        for (h = main_holes)
            translate([h[0] - main_w / 2, main_h / 2 - h[1], tray_t + 1])
                cylinder(d = m2_insert_d, h = 40);
        // cable pass-throughs to the outside mic and the right cup (headband cable)
        translate([-cup_a / 2 + 6, 0, -1]) cylinder(d = 5, h = 10);
        translate([cup_a / 2 - 6, 0, -1]) cylinder(d = 5, h = 10);
    }
}

// ---------------- front baffle: driver + error-mic arm ----------------
module baffle() {
    t = 2.0;
    difference() {
        union() {
            outline(t);
            translate([0, 0, 0]) cylinder(d = driver_d + 4, h = t + driver_depth * 0.5);
            // error-mic arm: puts the mic ~8 mm in front of the driver, near the ear canal
            translate([driver_d / 2 - 2, -mic_w / 2 - 1.2, 0]) cube([6, mic_w + 2.4, t + 9]);
        }
        translate([0, 0, -1]) cylinder(d = driver_d, h = 20);              // driver seat
        translate([0, 0, -1]) cylinder(d = driver_d - 3, h = 20);
        // mic board slot (plain back toward the ear, sound hole facing out)
        translate([driver_d / 2 - 0.5, -mic_w / 2 - 0.2, t + 1]) cube([pcb_t + 0.4, mic_w + 0.4, 20]);
        // rear-volume vent (tuning: tape over partially to change the driver's LF response)
        for (a = [45, 135, 225, 315]) rotate(a) translate([driver_d / 2 + 6, 0, -1]) cylinder(d = 3, h = 10);
        // wire slot
        translate([-driver_d / 2 - 6, -2, -1]) cube([4, 4, 10]);
    }
}

// layout for printing
if (part == "all") rear_tray();
if (part == "tray") rear_tray();
if (part == "all") translate([0, cup_b + 10, 0]) baffle();
if (part == "baffle") baffle();
