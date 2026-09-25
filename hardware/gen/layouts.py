"""Board outlines and hand-placed anchors (ICs, connectors, holes, UI parts).

Coordinates in mm from the board's top-left corner, rotation in degrees
(KiCad CCW). Connector footprints have their mouth toward +y at rot 0:
    rot 0 -> mouth down, 90 -> right, 180 -> up, 270 -> left.
Everything not anchored (passives, test points) is auto-placed next to the
pads it connects to (pcb_build.py).

MCU LQFP-100 pin sides (rot 0): 1-25 left (top->bottom), 26-50 bottom
(left->right), 51-75 right (bottom->top), 76-100 top (right->left).
ADC/DAC pins (28-34) exit bottom-left -> analog front ends bottom-left.
"""

LAYOUTS = {
    "main-board": {
        # 14 mm corners: the board has to sit inside an elliptical earcup (72 x 62 mm
        # inside, see mechanical/params.scad) - a 6 mm-corner rectangle doesn't fit.
        "size": (60.0, 50.0), "corner": 14.0, "layers": 4, "refs_on_fab": True,   # dense: refs on F.Fab, not silk
        "anchors": {
            # ref: (x, y, rot, side)
            "U5": (30.0, 25.0, 0, "F"),        # STM32H743 LQFP-100
            "J1": (41.0, 3.7, 180, "F"),        # USB-C, top edge
            "U1": (41.0, 10.8, 0, "F"),         # USBLC6
            "F1": (48.5, 8.5, 90, "F"),         # polyfuse
            "U2": (50.5, 14.5, 0, "F"),         # MCP73831
            "J2": (54.9, 20.0, 90, "F"),        # battery JST PH, right edge
            "Q1": (47.0, 18.5, 0, "F"),         # load-share P-FET
            "D2": (46.0, 14.0, 0, "F"),         # Schottky
            "D1": (55.0, 12.5, 90, "F"),        # charge LED
            "U3": (13.0, 8.0, 0, "F"),          # TPS63001
            "L1": (13.0, 3.6, 0, "F"),          # 2.2 uH
            "U4": (22.5, 10.5, 0, "F"),         # LP5907 2.8 V
            "Y1": (18.3, 24.5, 90, "F"),        # 25 MHz crystal next to PH0/PH1
            "J4": (32.0, 11.0, 0, "F"),         # SWD 2x5 1.27
            "J9": (24.5, 3.1, 180, "F"),        # BLE module header, top edge
            "U7": (10.5, 28.0, 90, "F"),        # REF L front end
            "U8": (10.5, 36.0, 90, "F"),        # ERR L front end
            "U6": (17.5, 36.0, 0, "F"),         # VMID buffer
            "J5": (3.4, 19.0, 270, "F"),        # left REF mic, left edge
            "J6": (3.4, 27.5, 270, "F"),        # left ERR mic, left edge
            "U9": (25.0, 40.0, 0, "F"),         # DAC reconstruction
            "U10": (33.0, 41.5, 0, "F"),        # TPA6132A2
            "J8": (39.0, 46.9, 0, "F"),         # left speaker, bottom edge
            "J7": (56.5, 30.5, 90, "F"),        # cable to right cup, right edge
            "SW1": (23.8, 46.4, 0, "B"),        # power button - back side: plunger runs straight to the shell
            "J3": (17.0, 46.9, 0, "F"),         # external button, bottom edge
            "SW2": (43.0, 31.5, 90, "F"),       # BOOT
            # status LEDs - back side, under the light pipe; rotated so the 0603 pads stack
            # along y (at rot 0 the 1.6 mm pad span doesn't fit the 2 mm pitch)
            "D4": (29.2, 47.6, 90, "B"),
            "D5": (31.2, 47.6, 90, "B"),
            "D6": (33.2, 47.6, 90, "B"),
            "H1": (7.2, 8.8, 0, "F"),
            "H2": (52.8, 42.8, 0, "F"),
            "H3": (6.8, 43.2, 0, "F"),
        },
    },
    "satellite-board": {
        "size": (34.0, 26.0), "corner": 5.0, "layers": 2,
        "anchors": {
            "J1": (17.0, 22.9, 0, "F"),         # cable to main, bottom edge
            "J3": (3.4, 9.5, 270, "F"),         # REF R mic, left edge
            "J4": (3.4, 18.5, 270, "F"),        # ERR R mic, left edge
            "J2": (30.8, 9.0, 90, "F"),         # right speaker, right edge
            "U2": (13.5, 9.0, 90, "F"),         # REF R front end
            "U3": (22.0, 9.0, 90, "F"),         # ERR R front end
            "U1": (25.5, 17.5, 0, "F"),         # VMID buffer
            "H1": (9.5, 3.5, 0, "F"),
            "H2": (30.0, 22.0, 0, "F"),
        },
    },
    "mic-board": {
        "size": (11.0, 13.2), "corner": 1.5, "layers": 2, "zone_full": True, "stitch_pitch": 1.6, "refs_on_fab": True,
        "anchors": {
            "MK1": (5.5, 3.6, 0, "F"),          # bottom-port mic: sound comes through the PCB hole
            "J1": (5.5, 9.65, 0, "F"),          # same side, cable exits the bottom edge; back stays flat
        },
        # the port's GND sealing ring is a custom pad the router won't enter: tie it to GND pad 4
        "hand_tracks": [("MK1", "5", "MK1", "4", 0.25)],
    },
}
