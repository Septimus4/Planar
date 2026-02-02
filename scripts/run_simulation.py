#!/usr/bin/env python3
"""Helper script to run the simulator and processing pipeline locally."""
import argparse
import os
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("--out", required=True)
parser.add_argument("--stations", type=int, default=3)
args = parser.parse_args()

session_dir = os.path.abspath(args.out)

# run simulator
subprocess.check_call(["python3", "-m", "simulation.generate_synthetic", "--out", session_dir, "--stations", str(args.stations)])
# run processing
outdir = os.path.join(session_dir, "artifacts")
subprocess.check_call(["python3", "-m", "processing.pipeline", "--session", session_dir, "--out", outdir])
print("Done. artifacts in:", outdir)
