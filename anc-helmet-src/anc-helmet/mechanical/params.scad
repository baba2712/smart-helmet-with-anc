// ANC helmet - shared mechanical parameters (mm).
// MEASURE YOUR EARMUFF and edit the "cup" block; everything else follows.

// ---- helmet-mount earmuff cup (inside, after removing the stock foam insert) ----
cup_a        = 72;    // inner long axis (vertical when worn)
cup_b        = 62;    // inner short axis
cup_depth    = 32;    // inner depth, rim to back wall
cup_wall     = 2.5;   // shell thickness where the outside mic sits

// ---- boards (must match hardware/gen/layouts.py; `python3 check_fit.py` verifies) ----
main_w = 60;  main_h = 50;  main_r = 14;
main_holes = [[7.2, 8.8], [52.8, 42.8], [6.8, 43.2]];  // from the board's top-left corner
sat_w  = 34;  sat_h  = 26;  sat_r  = 5;
sat_holes  = [[9.5, 3.5], [30, 22]];
mic_w  = 11;  mic_h  = 13.2;
mic_port   = [6.18, 3.6];                           // sound hole on the mic board (PCB underside)
pcb_t  = 1.6;

// ---- parts that live in the cups ----
driver_d      = 40.2;   // 40 mm headphone driver
driver_depth  = 6.0;
batt          = [50, 34, 10.5];   // 103450 LiPo pouch, 2000 mAh (with protection PCB)
m2_clear      = 2.3;
m2_insert_d   = 3.2;    // heat-set insert hole
standoff_h    = 4.0;

$fn = 64;

module rounded_rect(w, h, r, t) {
    linear_extrude(t) offset(r) offset(-r) square([w, h]);
}

module ellipse(a, b, t) { linear_extrude(t) scale([a / 2, b / 2]) circle(1); }
