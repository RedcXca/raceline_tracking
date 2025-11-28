#!/bin/bash
# Generate all modified track variants

for method in squeeze_x2 squeeze_y2 stretch_x2 stretch_y2; do
    echo "Generating $method variants..."
    for track in Montreal Monza IMS; do
        python modify_tracks.py $method racetracks/$track.csv racetracks/${track}_raceline.csv
    done
done

echo "All variants generated!"
