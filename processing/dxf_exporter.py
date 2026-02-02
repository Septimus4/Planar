"""DXF exporter for Planar floor plans.

This module exports processed scan data to DXF format for use in CAD software
like AutoCAD, Rhino, or similar applications.

Layers:
- WALLS: Extracted wall line segments
- DEBUG_POINTS: Sample of merged point cloud
- DEBUG_POSES: Station poses as markers
- ANNOTATIONS: Optional dimension and label annotations
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple, Optional, Sequence

import numpy as np

try:
    import ezdxf
    from ezdxf import colors
    from ezdxf.enums import TextEntityAlignment
    HAS_EZDXF = True
except ImportError:
    ezdxf = None  # type: ignore
    HAS_EZDXF = False

from .scan_matcher import Pose2D
from .wall_extractor import LineSegment


# Layer configuration
LAYER_WALLS = "WALLS"
LAYER_DEBUG_POINTS = "DEBUG_POINTS"
LAYER_DEBUG_POSES = "DEBUG_POSES"
LAYER_ANNOTATIONS = "ANNOTATIONS"

# Colors (AutoCAD Color Index)
COLOR_WALLS = 7  # White
COLOR_DEBUG_POINTS = 8  # Gray
COLOR_DEBUG_POSES = 1  # Red
COLOR_ANNOTATIONS = 3  # Green


@dataclass
class DxfExporter:
    """Export floor plan data to DXF format.
    
    Parameters:
        include_debug_points: Whether to include point cloud samples
        include_poses: Whether to include station pose markers
        include_annotations: Whether to include dimension annotations
        point_sample_rate: Sample every N points for debug layer
        pose_marker_size: Size of station pose markers in meters
    """
    
    include_debug_points: bool = True
    include_poses: bool = True
    include_annotations: bool = False
    point_sample_rate: int = 10  # Sample every 10th point
    pose_marker_size: float = 0.2  # meters
    point_marker_size: float = 0.02  # meters
    
    _doc: object = field(default=None, repr=False)
    _msp: object = field(default=None, repr=False)
    
    def __post_init__(self):
        """Initialize DXF document."""
        if not HAS_EZDXF:
            return
        
        self._doc = ezdxf.new(dxfversion="R2010")
        self._msp = self._doc.modelspace()
        
        # Create layers
        self._doc.layers.add(
            LAYER_WALLS,
            color=COLOR_WALLS,
            linetype="CONTINUOUS"
        )
        self._doc.layers.add(
            LAYER_DEBUG_POINTS,
            color=COLOR_DEBUG_POINTS,
            linetype="CONTINUOUS"
        )
        self._doc.layers.add(
            LAYER_DEBUG_POSES,
            color=COLOR_DEBUG_POSES,
            linetype="CONTINUOUS"
        )
        self._doc.layers.add(
            LAYER_ANNOTATIONS,
            color=COLOR_ANNOTATIONS,
            linetype="CONTINUOUS"
        )
    
    def add_walls(self, segments: Sequence[LineSegment]) -> None:
        """Add wall line segments to DXF.
        
        Args:
            segments: List of LineSegment objects
        """
        if not HAS_EZDXF or self._msp is None:
            return
        
        for seg in segments:
            self._msp.add_line(
                (seg.x1, seg.y1),
                (seg.x2, seg.y2),
                dxfattribs={"layer": LAYER_WALLS}
            )
    
    def add_wall_tuples(self, lines: Sequence[Tuple[float, float, float, float]]) -> None:
        """Add walls from (x1, y1, x2, y2) tuples.
        
        Args:
            lines: List of (x1, y1, x2, y2) tuples
        """
        if not HAS_EZDXF or self._msp is None:
            return
        
        for x1, y1, x2, y2 in lines:
            self._msp.add_line(
                (x1, y1),
                (x2, y2),
                dxfattribs={"layer": LAYER_WALLS}
            )
    
    def add_points(self, points: np.ndarray) -> None:
        """Add point cloud samples as small circles.
        
        Args:
            points: Nx2 array of (x, y) points
        """
        if not HAS_EZDXF or self._msp is None:
            return
        
        if not self.include_debug_points:
            return
        
        # Sample points
        step = max(1, self.point_sample_rate)
        sampled = points[::step]
        
        for x, y in sampled:
            self._msp.add_circle(
                (x, y),
                radius=self.point_marker_size,
                dxfattribs={"layer": LAYER_DEBUG_POINTS}
            )
    
    def add_poses(self, poses: Sequence[Pose2D], labels: Optional[Sequence[str]] = None) -> None:
        """Add station pose markers.
        
        Poses are rendered as arrows indicating position and heading.
        
        Args:
            poses: List of Pose2D objects
            labels: Optional labels for each pose (e.g., "S0", "S1", ...)
        """
        if not HAS_EZDXF or self._msp is None:
            return
        
        if not self.include_poses:
            return
        
        for i, pose in enumerate(poses):
            # Draw arrow at pose position pointing in heading direction
            x, y, theta = pose.x, pose.y, pose.theta
            
            # Arrow head
            head_len = self.pose_marker_size
            head_x = x + head_len * math.cos(theta)
            head_y = y + head_len * math.sin(theta)
            
            # Main line
            self._msp.add_line(
                (x, y),
                (head_x, head_y),
                dxfattribs={"layer": LAYER_DEBUG_POSES}
            )
            
            # Arrow wings
            wing_angle = math.radians(150)
            wing_len = head_len * 0.3
            
            for sign in [1, -1]:
                wing_x = head_x + wing_len * math.cos(theta + sign * wing_angle)
                wing_y = head_y + wing_len * math.sin(theta + sign * wing_angle)
                self._msp.add_line(
                    (head_x, head_y),
                    (wing_x, wing_y),
                    dxfattribs={"layer": LAYER_DEBUG_POSES}
                )
            
            # Station marker circle
            self._msp.add_circle(
                (x, y),
                radius=self.pose_marker_size * 0.5,
                dxfattribs={"layer": LAYER_DEBUG_POSES}
            )
            
            # Label
            if labels and i < len(labels):
                label = labels[i]
            else:
                label = f"S{i}"
            
            self._msp.add_text(
                label,
                height=self.pose_marker_size * 0.8,
                dxfattribs={
                    "layer": LAYER_DEBUG_POSES,
                    "insert": (x + self.pose_marker_size, y + self.pose_marker_size)
                }
            )
    
    def add_dimension(
        self,
        p1: Tuple[float, float],
        p2: Tuple[float, float],
        offset: float = 0.5
    ) -> None:
        """Add a linear dimension annotation.
        
        Args:
            p1: Start point (x, y)
            p2: End point (x, y)
            offset: Distance to offset dimension line from points
        """
        if not HAS_EZDXF or self._msp is None:
            return
        
        if not self.include_annotations:
            return
        
        # Calculate dimension line position
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.sqrt(dx*dx + dy*dy)
        
        if length < 0.01:
            return
        
        # Perpendicular direction
        perp_x = -dy / length
        perp_y = dx / length
        
        # Dimension line endpoints
        d1 = (p1[0] + offset * perp_x, p1[1] + offset * perp_y)
        d2 = (p2[0] + offset * perp_x, p2[1] + offset * perp_y)
        
        # Extension lines
        self._msp.add_line(p1, d1, dxfattribs={"layer": LAYER_ANNOTATIONS})
        self._msp.add_line(p2, d2, dxfattribs={"layer": LAYER_ANNOTATIONS})
        
        # Dimension line
        self._msp.add_line(d1, d2, dxfattribs={"layer": LAYER_ANNOTATIONS})
        
        # Dimension text
        mid = ((d1[0] + d2[0]) / 2, (d1[1] + d2[1]) / 2)
        text = f"{length:.2f}m"
        
        self._msp.add_text(
            text,
            height=0.1,
            dxfattribs={
                "layer": LAYER_ANNOTATIONS,
                "insert": mid,
                "rotation": math.degrees(math.atan2(dy, dx))
            }
        )
    
    def add_bounding_box(self, points: np.ndarray, margin: float = 0.5) -> None:
        """Add a bounding box around the point cloud.
        
        Args:
            points: Nx2 array of points
            margin: Margin to add around bounding box
        """
        if not HAS_EZDXF or self._msp is None:
            return
        
        if len(points) == 0:
            return
        
        x_min, y_min = np.min(points, axis=0) - margin
        x_max, y_max = np.max(points, axis=0) + margin
        
        # Rectangle
        corners = [
            (x_min, y_min),
            (x_max, y_min),
            (x_max, y_max),
            (x_min, y_max),
            (x_min, y_min)  # Close
        ]
        
        self._msp.add_lwpolyline(
            corners,
            dxfattribs={"layer": LAYER_ANNOTATIONS}
        )
    
    def save(self, path: Path | str) -> bool:
        """Save DXF to file.
        
        Args:
            path: Output file path
        
        Returns:
            True if successful
        """
        if not HAS_EZDXF or self._doc is None:
            # Fallback: write stub file
            with open(path, 'w') as f:
                f.write("# ezdxf not installed; cannot write DXF\n")
            return False
        
        self._doc.saveas(path)
        return True


def export_floor_plan(
    walls: Sequence[LineSegment],
    points: Optional[np.ndarray] = None,
    poses: Optional[Sequence[Pose2D]] = None,
    output_path: Path | str = "floor_plan.dxf",
    **kwargs
) -> bool:
    """Convenience function to export a complete floor plan.
    
    Args:
        walls: Wall line segments
        points: Optional merged point cloud
        poses: Optional station poses
        output_path: Output DXF file path
        **kwargs: Additional parameters for DxfExporter
    
    Returns:
        True if successful
    """
    exporter = DxfExporter(**kwargs)
    
    exporter.add_walls(walls)
    
    if points is not None and len(points) > 0:
        exporter.add_points(points)
    
    if poses is not None and len(poses) > 0:
        exporter.add_poses(poses)
    
    return exporter.save(output_path)


# Legacy function for backward compatibility
def export_walls(lines: Sequence[Tuple[float, float, float, float]], outpath: str) -> None:
    """Export wall lines to DXF (legacy interface).
    
    Args:
        lines: List of (x1, y1, x2, y2) tuples
        outpath: Output file path
    """
    if not HAS_EZDXF:
        with open(outpath, "w") as f:
            f.write("# ezdxf not installed; cannot write DXF\n")
        return
    
    doc = ezdxf.new(dxfversion="R2010")
    msp = doc.modelspace()
    
    for (x1, y1, x2, y2) in lines:
        msp.add_line((x1, y1), (x2, y2), dxfattribs={"layer": LAYER_WALLS})
    
    doc.saveas(outpath)

