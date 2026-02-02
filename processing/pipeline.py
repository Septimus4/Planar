"""Processing pipeline for Planar floor plan scanner.

This module implements the complete processing pipeline:
1. Load session data (LiDAR stations, IMU logs, events)
2. Compute IMU yaw priors for registration
3. Register stations using ICP with yaw priors
4. Optimize pose graph for global consistency
5. Merge point clouds into unified coordinate frame
6. Extract wall line segments
7. Export to DXF

Usage:
    python -m processing.pipeline --session sessions/my_session --out artifacts/output

The pipeline applies quality gates at each stage and produces a detailed summary.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Dict, Any

import numpy as np

from .imu_processor import ImuProcessor, YawPrior, load_yaw_priors
from .scan_matcher import ScanMatcher, Pose2D, RegistrationResult, register_stations
from .pose_graph import PoseGraph, build_pose_graph, PoseGraphEdge
from .wall_extractor import WallExtractor, LineSegment, extract_walls
from .dxf_exporter import DxfExporter, export_floor_plan

try:
    import ezdxf
    HAS_EZDXF = True
except ImportError:
    HAS_EZDXF = False


@dataclass
class QualityGates:
    """Quality thresholds for pipeline stages."""
    
    # Registration thresholds
    min_fitness: float = 0.3  # Minimum ICP fitness score
    max_rmse: float = 0.2  # Maximum registration RMSE (meters)
    max_translation: float = 10.0  # Maximum translation between stations (meters)
    max_rotation_deg: float = 180.0  # Maximum rotation between stations
    
    # Wall extraction thresholds
    min_wall_length: float = 0.3  # Minimum wall segment length (meters)
    min_wall_count: int = 3  # Minimum number of walls to consider valid
    
    # Point cloud thresholds
    min_points_per_station: int = 100  # Minimum points per station


@dataclass
class PipelineResult:
    """Result of the processing pipeline."""
    
    success: bool
    session_path: str
    output_path: str
    
    # Processing stats
    num_stations: int = 0
    num_points_total: int = 0
    num_walls: int = 0
    
    # Quality metrics
    registrations: List[Dict[str, Any]] = field(default_factory=list)
    pose_graph_cost: float = 0.0
    
    # Issues and warnings
    warnings: List[str] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)
    
    # Timing
    processing_time_sec: float = 0.0
    
    def to_dict(self) -> dict:
        """Convert to dictionary for JSON serialization."""
        def convert_value(v):
            """Convert numpy types to Python native types."""
            if isinstance(v, (np.bool_, np.integer)):
                return int(v)
            elif isinstance(v, np.floating):
                return float(v)
            elif isinstance(v, np.ndarray):
                return v.tolist()
            elif isinstance(v, dict):
                return {k: convert_value(vv) for k, vv in v.items()}
            elif isinstance(v, list):
                return [convert_value(vv) for vv in v]
            return v
        
        d = asdict(self)
        return convert_value(d)


@dataclass
class ProcessingPipeline:
    """Main processing pipeline for Planar sessions.
    
    The pipeline processes a captured session through:
    1. Data loading
    2. IMU processing
    3. Scan registration  
    4. Pose graph optimization
    5. Point cloud merging
    6. Wall extraction
    7. DXF export
    """
    
    quality_gates: QualityGates = field(default_factory=QualityGates)
    
    # Module instances
    imu_processor: ImuProcessor = field(default_factory=ImuProcessor)
    scan_matcher: ScanMatcher = field(default_factory=ScanMatcher)
    wall_extractor: WallExtractor = field(default_factory=WallExtractor)
    
    # Processing state
    session_dir: Optional[Path] = None
    output_dir: Optional[Path] = None
    
    # Data
    station_paths: List[Path] = field(default_factory=list)
    station_points: List[np.ndarray] = field(default_factory=list)
    yaw_priors: List[YawPrior] = field(default_factory=list)
    poses: List[Pose2D] = field(default_factory=list)
    registration_results: List[RegistrationResult] = field(default_factory=list)
    merged_points: Optional[np.ndarray] = None
    walls: List[LineSegment] = field(default_factory=list)
    
    # Result tracking
    result: PipelineResult = field(default_factory=lambda: PipelineResult(
        success=False, session_path="", output_path=""
    ))
    
    def load_session(self, session_dir: Path | str) -> bool:
        """Load session data from directory.
        
        Args:
            session_dir: Path to session directory
        
        Returns:
            True if loading successful
        """
        self.session_dir = Path(session_dir)
        self.result.session_path = str(self.session_dir)
        
        if not self.session_dir.exists():
            self.result.errors.append(f"Session directory not found: {session_dir}")
            return False
        
        # Find station files
        self.station_paths = sorted(
            self.session_dir.glob("lidar_station_*.csv"),
            key=lambda p: int(p.stem.split("_")[-1])
        )
        
        if len(self.station_paths) == 0:
            self.result.errors.append("No station files found in session")
            return False
        
        self.result.num_stations = len(self.station_paths)
        
        # Load station point clouds
        self.station_points = []
        for path in self.station_paths:
            points = self.scan_matcher.load_station(path)
            if len(points) < self.quality_gates.min_points_per_station:
                self.result.warnings.append(
                    f"Station {path.name} has only {len(points)} points "
                    f"(minimum: {self.quality_gates.min_points_per_station})"
                )
            self.station_points.append(points)
            self.result.num_points_total += len(points)
        
        # Load IMU data if available
        imu_log = self.session_dir / "imu_log.csv"
        events_file = self.session_dir / "events.json"
        
        if imu_log.exists():
            self.imu_processor.load_imu_log(imu_log)
            
            if events_file.exists():
                self.imu_processor.load_events(events_file)
                self.yaw_priors = self.imu_processor.compute_yaw_priors()
        else:
            self.result.warnings.append("No IMU log found; using registration without priors")
        
        return True
    
    def register_stations(self) -> bool:
        """Register all stations to create unified coordinate frame.
        
        Returns:
            True if all registrations successful
        """
        if len(self.station_points) == 0:
            self.result.errors.append("No station points loaded")
            return False
        
        # First station at origin
        self.poses = [Pose2D()]
        self.registration_results = []
        
        all_success = True
        
        for i in range(1, len(self.station_points)):
            source = self.station_points[i]
            target = self.station_points[i - 1]
            
            # Get yaw prior if available
            yaw_prior = None
            yaw_uncertainty = None
            if i - 1 < len(self.yaw_priors):
                prior = self.yaw_priors[i - 1]
                yaw_prior = prior.delta_yaw_rad
                yaw_uncertainty = prior.uncertainty_rad
            
            # Register
            result = self.scan_matcher.register(
                source, target,
                yaw_prior=yaw_prior,
                yaw_uncertainty=yaw_uncertainty
            )
            
            self.registration_results.append(result)
            
            # Validate
            validation = self.scan_matcher.validate_registration(result)
            
            reg_info = {
                "station": i,
                "fitness": result.fitness,
                "rmse": result.rmse,
                "converged": result.converged,
                "translation_m": validation["translation"],
                "rotation_deg": validation["rotation_deg"],
                "valid": validation["valid"],
                "issues": validation["issues"]
            }
            self.result.registrations.append(reg_info)
            
            if not validation["valid"]:
                self.result.warnings.append(
                    f"Registration station {i}: " + "; ".join(validation["issues"])
                )
                all_success = False
            
            # Compose pose (transform is source->target, we want global pose of source)
            global_pose = self.poses[-1].compose(result.transform.inverse())
            self.poses.append(global_pose)
        
        return all_success
    
    def optimize_poses(self) -> bool:
        """Optimize pose graph for global consistency.
        
        Returns:
            True if optimization successful
        """
        if len(self.poses) < 2:
            return True  # Nothing to optimize
        
        # Build pose graph
        graph = build_pose_graph(
            self.poses,
            self.registration_results,
            self.yaw_priors
        )
        
        # Optimize
        opt_result = graph.optimize()
        
        if opt_result["success"]:
            self.poses = opt_result["poses"]
            self.result.pose_graph_cost = opt_result.get("cost", 0.0)
            return True
        else:
            self.result.warnings.append(
                f"Pose graph optimization failed: {opt_result.get('message', 'unknown')}"
            )
            return False
    
    def merge_point_clouds(self) -> bool:
        """Merge all stations into unified coordinate frame.
        
        Returns:
            True if merging successful
        """
        all_points = []
        
        for i, (points, pose) in enumerate(zip(self.station_points, self.poses)):
            # Transform points to global frame
            transformed = self.scan_matcher.transform_points(points, pose)
            all_points.append(transformed)
        
        if all_points:
            self.merged_points = np.vstack(all_points)
            self.result.num_points_total = len(self.merged_points)
            return True
        
        self.result.errors.append("No points to merge")
        return False
    
    def extract_walls(self) -> bool:
        """Extract wall line segments from merged point cloud.
        
        Returns:
            True if extraction successful
        """
        if self.merged_points is None or len(self.merged_points) == 0:
            self.result.errors.append("No merged points for wall extraction")
            return False
        
        self.walls = self.wall_extractor.extract_walls(self.merged_points)
        self.result.num_walls = len(self.walls)
        
        if len(self.walls) < self.quality_gates.min_wall_count:
            self.result.warnings.append(
                f"Only {len(self.walls)} walls extracted "
                f"(minimum: {self.quality_gates.min_wall_count})"
            )
        
        return len(self.walls) > 0
    
    def export_results(self, output_dir: Path | str) -> bool:
        """Export all results to output directory.
        
        Args:
            output_dir: Output directory path
        
        Returns:
            True if export successful
        """
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.result.output_path = str(self.output_dir)
        
        # Export merged point cloud as CSV
        if self.merged_points is not None:
            merged_path = self.output_dir / "merged.csv"
            with open(merged_path, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["x", "y"])
                for x, y in self.merged_points:
                    writer.writerow([f"{x:.6f}", f"{y:.6f}"])
        
        # Export walls as CSV
        walls_csv_path = self.output_dir / "walls.csv"
        with open(walls_csv_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["x1", "y1", "x2", "y2", "length"])
            for wall in self.walls:
                writer.writerow([
                    f"{wall.x1:.6f}",
                    f"{wall.y1:.6f}",
                    f"{wall.x2:.6f}",
                    f"{wall.y2:.6f}",
                    f"{wall.length:.6f}"
                ])
        
        # Export poses as CSV
        poses_path = self.output_dir / "poses.csv"
        with open(poses_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["station", "x", "y", "theta_rad", "theta_deg"])
            for i, pose in enumerate(self.poses):
                writer.writerow([
                    i,
                    f"{pose.x:.6f}",
                    f"{pose.y:.6f}",
                    f"{pose.theta:.6f}",
                    f"{math.degrees(pose.theta):.2f}"
                ])
        
        # Export DXF
        dxf_path = self.output_dir / "walls.dxf"
        export_floor_plan(
            walls=self.walls,
            points=self.merged_points,
            poses=self.poses,
            output_path=dxf_path,
            include_debug_points=True,
            include_poses=True
        )
        
        # Export summary JSON
        summary_path = self.output_dir / "summary.json"
        with open(summary_path, "w") as f:
            json.dump(self.result.to_dict(), f, indent=2)
        
        return True
    
    def run(self, session_dir: Path | str, output_dir: Path | str) -> PipelineResult:
        """Run the complete processing pipeline.
        
        Args:
            session_dir: Path to session directory
            output_dir: Path to output directory
        
        Returns:
            PipelineResult with processing outcomes
        """
        import time
        start_time = time.time()
        
        self.result = PipelineResult(
            success=False,
            session_path=str(session_dir),
            output_path=str(output_dir)
        )
        
        # Step 1: Load session
        print(f"Loading session from {session_dir}...")
        if not self.load_session(session_dir):
            return self.result
        print(f"  Found {self.result.num_stations} stations, {self.result.num_points_total} points")
        
        # Step 2: Register stations
        print("Registering stations...")
        if not self.register_stations():
            print("  Warning: Some registrations had issues")
        print(f"  Registered {len(self.poses)} stations")
        
        # Step 3: Optimize pose graph
        print("Optimizing pose graph...")
        self.optimize_poses()
        print(f"  Final cost: {self.result.pose_graph_cost:.4f}")
        
        # Step 4: Merge point clouds
        print("Merging point clouds...")
        if not self.merge_point_clouds():
            return self.result
        print(f"  Merged {len(self.merged_points)} points")
        
        # Step 5: Extract walls
        print("Extracting walls...")
        if not self.extract_walls():
            self.result.warnings.append("Wall extraction failed; proceeding without walls")
        print(f"  Extracted {len(self.walls)} walls")
        
        # Step 6: Export results
        print(f"Exporting results to {output_dir}...")
        if not self.export_results(output_dir):
            return self.result
        print("  Export complete")
        
        self.result.processing_time_sec = time.time() - start_time
        self.result.success = len(self.result.errors) == 0
        
        return self.result


def run(session_dir: str, outdir: str) -> None:
    """Legacy interface for running the pipeline.
    
    Args:
        session_dir: Path to session directory
        outdir: Path to output directory
    """
    pipeline = ProcessingPipeline()
    result = pipeline.run(session_dir, outdir)
    
    # Print summary to stdout
    print(json.dumps(result.to_dict(), indent=2))


# Legacy functions for backward compatibility
def read_session(session_dir: str) -> List[str]:
    """Read station file paths from session directory."""
    stations = []
    for fname in sorted(os.listdir(session_dir)):
        if fname.startswith("lidar_station_") and fname.endswith(".csv"):
            stations.append(os.path.join(session_dir, fname))
    return stations


def load_station_csv(path: str) -> List[tuple]:
    """Load station points from CSV file."""
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


def write_merged_csv(points: List[tuple], outpath: str) -> None:
    """Write merged points to CSV file."""
    with open(outpath, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y"])
        for x, y in points:
            w.writerow([f"{x:.6f}", f"{y:.6f}"])


def write_stub_dxf(points: List[tuple], outpath: str) -> None:
    """Write a stub DXF file with debug points."""
    if not HAS_EZDXF:
        with open(outpath, "w") as f:
            f.write("# ezdxf not installed; stub DXF\n")
        return
    doc = ezdxf.new(dxfversion="R2010")
    msp = doc.modelspace()
    for (x, y) in points[::max(1, len(points) // 1000)]:
        msp.add_circle((x, y), radius=0.02, dxfattribs={"layer": "DEBUG_POINTS"})
    doc.saveas(outpath)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process Planar session into floor plan DXF"
    )
    parser.add_argument(
        "--session", 
        required=True,
        help="Path to session directory"
    )
    parser.add_argument(
        "--out",
        required=True,
        help="Path to output directory"
    )
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Enable verbose output"
    )
    
    args = parser.parse_args()
    
    pipeline = ProcessingPipeline()
    result = pipeline.run(args.session, args.out)
    
    # Print summary
    print("\n" + "=" * 60)
    print("PROCESSING SUMMARY")
    print("=" * 60)
    print(f"Status: {'SUCCESS' if result.success else 'FAILED'}")
    print(f"Stations: {result.num_stations}")
    print(f"Points: {result.num_points_total}")
    print(f"Walls: {result.num_walls}")
    print(f"Time: {result.processing_time_sec:.2f}s")
    
    if result.warnings:
        print(f"\nWarnings ({len(result.warnings)}):")
        for w in result.warnings:
            print(f"  - {w}")
    
    if result.errors:
        print(f"\nErrors ({len(result.errors)}):")
        for e in result.errors:
            print(f"  - {e}")
    
    sys.exit(0 if result.success else 1)
