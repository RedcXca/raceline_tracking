#!/bin/bash
# Runs all tracks in headless mode (steps=0) and writes output to test.out
# Usage: ./run_all_headless.sh

OUT_FILE="test.out"
> "$OUT_FILE"  # truncate/create

echo "Running Montreal (headless)" | tee -a "$OUT_FILE"
python main.py ./racetracks/Montreal.csv ./racetracks/Montreal_raceline.csv 0 >> "$OUT_FILE" 2>&1

echo "Running Monza (headless)" | tee -a "$OUT_FILE"
python main.py ./racetracks/Monza.csv ./racetracks/Monza_raceline.csv 0 >> "$OUT_FILE" 2>&1

echo "Running IMS (headless)" | tee -a "$OUT_FILE"
python main.py ./racetracks/IMS.csv ./racetracks/IMS_raceline.csv 0 >> "$OUT_FILE" 2>&1

echo "Done. Output saved to $OUT_FILE" | tee -a "$OUT_FILE"
