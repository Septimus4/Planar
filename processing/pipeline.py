"""Minimal processing pipeline entrypoints for Planar.

This module implements a small CLI that reads a session directory produced by the simulator
and writes a merged point cloud (as CSV) and a stub DXF file (via ezdxf) to the output dir.
This is intentionally minimal so it can be expanded by agents.
"""
import argparse
import os
import json
import csv
import math

try:
    import ezdxf
except Exception:
    ezdxf = None


def read_session(session_dir):
    stations = []
    for fname in sorted(os.listdir(session_dir)):
        if fname.startswith("lidar_station_") and fname.endswith(".csv"):
            stations.append(os.path.join(session_dir, fname))
    return stations


def load_station_csv(path):
    pts = []
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        for row in r:
            a = float(row.get("angle_deg", row.get("angle", 0)))
            rdist = float(row.get("range_m", row.get("range", 0)))
            rad = math.radians(a)
            x = rdist * math.cos(rad)
            y = rdist * math.sin(rad)
            pts.append((x, y))
    return pts


def write_merged_csv(points, outpath):
    with open(outpath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for x, y in points:
            w.writerow([f"{x:.6f}", f"{y:.6f}"])


def write_stub_dxf(points, outpath):
    if ezdxf is None:
        with open(outpath, "w") as f:
            f.write("# ezdxf not installed; stub DXF\n")
        return
    doc = ezdxf.new(dxfversion="R2010")
    msp = doc.modelspace()
    # write points as short lines (for debug)
    for (x, y) in points[:: max(1, len(points)//1000)]:
        msp.add_circle((x, y), radius=0.02, dxfattribs={"layer": "DEBUG_POINTS"})
    doc.saveas(outpath)


def run(session_dir, outdir):
    os.makedirs(outdir, exist_ok=True)
    stations = read_session(session_dir)
    merged = []
    for s in stations:
        pts = load_station_csv(s)
        merged.extend(pts)
    merged_path = os.path.join(outdir, "merged.csv")
    write_merged_csv(merged, merged_path)
    dxf_path = os.path.join(outdir, "walls.dxf")
    write_stub_dxf(merged, dxf_path)
    summary = {"status": "ok", "stations": len(stations), "points": len(merged)}
    with open(os.path.join(outdir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    print(json.dumps(summary))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    run(args.session, args.out)
