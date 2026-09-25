// Outside (reference) mic pod: glued/screwed onto the outside of the cup shell.
// The mic board sits plain-side-out behind a mesh window; the recess takes a
// 3 mm open-cell foam disc as a wind screen (wind noise wrecks feed-forward ANC).
include <params.scad>

part = "all";   // openscad -D 'part="..."' exports one part

wall = 1.4;
pod  = [mic_w + 2 * wall + 1, mic_h + 2 * wall + 1, 7];

module ref_mic_pod() {
    difference() {
        hull() {
            translate([0, 0, 0]) cube([pod[0], pod[1], 1]);
            translate([1.5, 1.5, pod[2] - 1]) cube([pod[0] - 3, pod[1] - 3, 1]);
        }
        // board pocket (opens to the cup side)
        translate([wall, wall, -0.01]) cube([mic_w + 1, mic_h + 1, pcb_t + 3.2]);
        // acoustic port in front of the PCB sound hole + foam recess
        translate([wall + 0.5 + mic_port[0], wall + 0.5 + (mic_h - mic_port[1]), 0]) {
            cylinder(d = 2.0, h = 20);
            translate([0, 0, pcb_t + 3.2 + 0.8]) cylinder(d = 8, h = 20);
        }
        // cable exit through the shell
        translate([pod[0] / 2 - 2.5, -1, -0.01]) cube([5, wall + 2, 2.4]);
    }
}

// drill guide for the shell: marks the cable hole and the two pod-screw holes
module drill_template() {
    difference() {
        cube([pod[0] + 10, pod[1] + 10, 1.2]);
        translate([5 + pod[0] / 2, 5 + 1, -1]) cylinder(d = 5, h = 5);
        translate([2.5, (pod[1] + 10) / 2, -1]) cylinder(d = 2, h = 5);
        translate([pod[0] + 7.5, (pod[1] + 10) / 2, -1]) cylinder(d = 2, h = 5);
    }
}

if (part == "all") ref_mic_pod();
if (part == "pod") ref_mic_pod();
if (part == "all") translate([pod[0] + 8, 0, 0]) drill_template();
if (part == "template") drill_template();
