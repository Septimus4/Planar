"""Planar processing pipeline package.

This package provides the complete processing pipeline for converting
captured LiDAR sessions into floor plan DXF files.

Modules:
- pipeline: Main processing orchestration
- imu_processor: IMU yaw prior computation
- scan_matcher: ICP scan registration
- pose_graph: Pose graph optimization
- wall_extractor: RANSAC wall extraction
- dxf_exporter: DXF file export
"""

from .pipeline import ProcessingPipeline, PipelineResult, QualityGates, run
from .imu_processor import ImuProcessor, ImuSample, YawPrior, load_yaw_priors
from .scan_matcher import ScanMatcher, Pose2D, RegistrationResult, register_stations
from .pose_graph import PoseGraph, PoseGraphEdge, build_pose_graph
from .wall_extractor import WallExtractor, LineSegment, extract_walls
from .dxf_exporter import DxfExporter, export_floor_plan, export_walls

__all__ = [
    # Pipeline
    "ProcessingPipeline",
    "PipelineResult", 
    "QualityGates",
    "run",
    # IMU
    "ImuProcessor",
    "ImuSample",
    "YawPrior",
    "load_yaw_priors",
    # Scan matching
    "ScanMatcher",
    "Pose2D",
    "RegistrationResult",
    "register_stations",
    # Pose graph
    "PoseGraph",
    "PoseGraphEdge",
    "build_pose_graph",
    # Wall extraction
    "WallExtractor",
    "LineSegment",
    "extract_walls",
    # DXF export
    "DxfExporter",
    "export_floor_plan",
    "export_walls",
]
