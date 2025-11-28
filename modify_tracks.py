#!/usr/bin/env python3
import csv
import os
import sys
from pathlib import Path

METHODS = {
    # squeeze by 2 ==> multiply by 0.5
    "squeeze_x2": (0.5, 1.0),
    "squeeze_y2": (1.0, 0.5),
    "stretch_x2": (2.0, 1.0),
    "stretch_y2": (1.0, 2.0),
}

def read_points(csv_path: Path):
    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            try:
                # Parse all columns, keep extras (like width)
                parsed = [float(val) for val in row]
                rows.append(parsed)
            except (ValueError, IndexError):
                # Skip rows that can't be parsed
                continue
    return rows

def write_points(csv_path: Path, rows):
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        for row in rows:
            writer.writerow(row)

def apply_scale(rows, sx: float, sy: float):
    # Scale first two columns (x, y), preserve rest
    scaled = []
    for row in rows:
        new_row = [row[0] * sx, row[1] * sy] + row[2:]
        scaled.append(new_row)
    return scaled

def process_file(in_path: Path, method: str, out_dir: Path):
    base = in_path.stem  # filename without extension
    sx, sy = METHODS[method]
    pts = read_points(in_path)
    scaled = apply_scale(pts, sx, sy)
    out_name = f"{method}_{base}.csv"
    out_path = out_dir / out_name
    write_points(out_path, scaled)
    return out_path

def main():
    if len(sys.argv) < 3:
        print("Usage: python modify_tracks.py <method> <input_csv1> [<input_csv2> ...]")
        print("Methods:", ", ".join(METHODS.keys()))
        print("Example: python modify_tracks.py squeeze_x2 racetracks/Montreal.csv racetracks/Montreal_raceline.csv")
        sys.exit(1)

    method = sys.argv[1]
    if method not in METHODS:
        print(f"Unknown method: {method}")
        print("Methods:", ", ".join(METHODS.keys()))
        sys.exit(1)

    inputs = [Path(p) for p in sys.argv[2:]]
    out_dir = Path("racetracks")  # write alongside existing

    for inp in inputs:
        out_path = process_file(inp, method, out_dir)
        print(f"Wrote: {out_path}")

if __name__ == "__main__":
    main()
