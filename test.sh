#!/bin/bash
# Runs all track pairs in racetracks/ in headless mode and writes output to test.out
# Pairs are matched as: <basename>.csv with <basename>_raceline.csv
# Usage: ./test.sh

OUT_FILE="test.out"
> "$OUT_FILE"  # truncate/create

echo "=== Testing All Tracks ===" | tee -a "$OUT_FILE"

# Find all *_raceline.csv files
for raceline in racetracks/*_raceline.csv; do
    # Extract basename (e.g., "Montreal" from "Montreal_raceline.csv")
    base=$(basename "$raceline" _raceline.csv)
    track="racetracks/${base}.csv"
    
    # Check if corresponding track file exists
    if [[ -f "$track" ]]; then
        echo "Running $base (headless)" | tee -a "$OUT_FILE"
        python main.py "$track" "$raceline" 0 >> "$OUT_FILE" 2>&1
    fi
done

echo "" | tee -a "$OUT_FILE"
echo "Done. Output saved to $OUT_FILE" | tee -a "$OUT_FILE"
