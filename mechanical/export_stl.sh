#!/bin/sh
# Export every printable part to stl/ (needs OpenSCAD). Re-run after editing params.scad
# with your measured earmuff, and after `python3 check_fit.py` passes.
#   left cup : cup_insert_left-tray, cup_insert_left-baffle
#   right cup: cup_insert_right-tray, cup_insert_right-baffle
#   outside  : mic_pods-pod (print 2), mic_pods-template (drill guide for the shell)
#   UI       : light_pipe_button (clear PETG light pipe + TPU plunger, split in the slicer)
set -e
cd "$(dirname "$0")"
mkdir -p stl
for job in cup_insert_left:tray cup_insert_left:baffle cup_insert_right:tray cup_insert_right:baffle \
           mic_pods:pod mic_pods:template light_pipe_button:all; do
    f=${job%%:*}; p=${job##*:}
    out=stl/$f-$p.stl
    [ "$p" = all ] && out=stl/$f.stl
    openscad -q -o "$out" -D "part=\"$p\"" "$f.scad"
    echo "  $out"
done
