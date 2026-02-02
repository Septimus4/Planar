"""Simple synthetic session generator for Planar.

This module produces a minimal session directory with:
- lidar_station_0.csv, lidar_station_1.csv, ...
- imu_log.csv
- events.json
- metadata.json

The output format is intentionally simple so processing-agent can read it in tests.
"""
import json
import math
import os
import argparse
import csv
import random
from datetime import datetime


def make_session(outdir, n_stations=3, points_per_station=360):
    os.makedirs(outdir, exist_ok=True)
    metadata = {
        "project": "Planar-sim",
        "created": datetime.utcnow().isoformat() + "Z",
        "stations": n_stations,
    }
    with open(os.path.join(outdir, "metadata.json"), "w") as f:
        json.dump(metadata, f, indent=2)

    events = []
    imu_samples = []

    # simple square room
    walls = [(-5, -5, 5, -5), (5, -5, 5, 5), (5, 5, -5, 5), (-5, 5, -5, -5)]

    for s in range(n_stations):
        station_file = os.path.join(outdir, f"lidar_station_{s}.csv")
        cx = -2 + s * 2  # place stations along x
        cy = 0
        with open(station_file, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "angle_deg", "range_m"])
            for i in range(points_per_station):
                angle = i * 360.0 / points_per_station
                rad = math.radians(angle)
                # cast ray and find nearest wall intersection (very simple)
                r = 10.0
                for (x1, y1, x2, y2) in walls:
                    # parameterize line segment and ray; approximate by sampling direction
                    # For simplicity, put range to boundary based on quadrant
                    pass
                # instead use a rectangular room model analytic
                # ray from (cx,cy)
                dx = math.cos(rad)
                dy = math.sin(rad)
                t_candidates = []
                # vertical walls x = -5 and x = 5
                for xw in (-5, 5):
                    if abs(dx) > 1e-6:
                        t = (xw - cx) / dx
                        if t > 0:
                            y_hit = cy + t * dy
                            if -5 - 1e-6 <= y_hit <= 5 + 1e-6:
                                t_candidates.append(t)
                # horizontal walls y = -5 and y = 5
                for yw in (-5, 5):
                    if abs(dy) > 1e-6:
                        t = (yw - cy) / dy
                        if t > 0:
                            x_hit = cx + t * dx
                            if -5 - 1e-6 <= x_hit <= 5 + 1e-6:
                                t_candidates.append(t)
                if t_candidates:
                    r = min(t_candidates)
                # add noise
                r += random.gauss(0, 0.01)
                ts = s * 10 + i * 0.01
                writer.writerow([f"{ts:.6f}", f"{angle:.3f}", f"{r:.6f}"])

        events.append({"type": "station_captured", "station": s, "timestamp": s * 10})

        # produce some imu samples: stationary during capture
        for k in range(100):
            imu_samples.append({"t": s * 10 + k * 0.01, "gyro_z": random.gauss(0.0, 0.001)})

    # write imu_log.csv
    with open(os.path.join(outdir, "imu_log.csv"), "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["timestamp", "gyro_z_rad_s"])
        for s in imu_samples:
            writer.writerow([f"{s['t']:.6f}", f"{s['gyro_z']:.6f}"])

    with open(os.path.join(outdir, "events.json"), "w") as f:
        json.dump(events, f, indent=2)

    print(json.dumps({"status": "ok", "session_dir": outdir}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="Output session directory")
    parser.add_argument("--stations", type=int, default=3)
    args = parser.parse_args()
    make_session(args.out, n_stations=args.stations)
