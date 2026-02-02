"""Wall extraction from merged point cloud.

This module implements wall line segment extraction using:
1. DBSCAN clustering to separate wall clusters
2. RANSAC line fitting within clusters
3. Segment merging for collinear segments
4. Endpoint refinement and snapping
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np

try:
    from sklearn.cluster import DBSCAN
    from sklearn.linear_model import RANSACRegressor
    HAS_SKLEARN = True
except ImportError:
    HAS_SKLEARN = False


@dataclass
class LineSegment:
    """2D line segment."""
    x1: float
    y1: float
    x2: float
    y2: float
    
    @property
    def length(self) -> float:
        """Segment length."""
        return math.sqrt((self.x2 - self.x1)**2 + (self.y2 - self.y1)**2)
    
    @property
    def angle(self) -> float:
        """Angle in radians from +X axis."""
        return math.atan2(self.y2 - self.y1, self.x2 - self.x1)
    
    @property
    def midpoint(self) -> Tuple[float, float]:
        """Segment midpoint."""
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)
    
    def point_distance(self, x: float, y: float) -> float:
        """Distance from a point to the infinite line through this segment."""
        # Line: ax + by + c = 0
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        length = self.length
        if length < 1e-9:
            return math.sqrt((x - self.x1)**2 + (y - self.y1)**2)
        
        # Normal form
        a = -dy / length
        b = dx / length
        c = -(a * self.x1 + b * self.y1)
        
        return abs(a * x + b * y + c)
    
    def project_point(self, x: float, y: float) -> Tuple[float, float]:
        """Project a point onto the infinite line through this segment."""
        dx = self.x2 - self.x1
        dy = self.y2 - self.y1
        length_sq = dx*dx + dy*dy
        
        if length_sq < 1e-9:
            return self.x1, self.y1
        
        # Project point onto line parameter t
        t = ((x - self.x1) * dx + (y - self.y1) * dy) / length_sq
        
        return self.x1 + t * dx, self.y1 + t * dy
    
    def to_tuple(self) -> Tuple[float, float, float, float]:
        """Return as (x1, y1, x2, y2) tuple."""
        return (self.x1, self.y1, self.x2, self.y2)


@dataclass
class WallExtractor:
    """Extract wall line segments from a 2D point cloud.
    
    Pipeline:
    1. Cluster points using DBSCAN
    2. For each cluster, fit lines using RANSAC
    3. Merge collinear segments
    4. Snap endpoints to create clean intersections
    """
    
    # DBSCAN parameters
    cluster_eps: float = 0.15  # meters - max distance between points in cluster
    cluster_min_samples: int = 5  # minimum points to form cluster
    
    # RANSAC parameters  
    ransac_threshold: float = 0.05  # meters - max distance from line to be inlier
    ransac_min_samples: int = 10  # minimum inliers to accept line
    min_segment_length: float = 0.3  # meters - minimum wall segment length
    
    # Merging parameters
    merge_angle_threshold: float = math.radians(5)  # max angle difference to merge
    merge_distance_threshold: float = 0.2  # meters - max perpendicular distance
    merge_gap_threshold: float = 0.5  # meters - max gap between segments to merge
    
    # Snapping parameters
    snap_distance: float = 0.1  # meters - snap endpoints within this distance
    
    def cluster_points(self, points: np.ndarray) -> List[np.ndarray]:
        """Cluster points using DBSCAN.
        
        Args:
            points: Nx2 array of (x, y) points
        
        Returns:
            List of point arrays, one per cluster
        """
        if len(points) < self.cluster_min_samples:
            return [points] if len(points) > 0 else []
        
        if not HAS_SKLEARN:
            # Fallback: treat all points as one cluster
            return [points]
        
        clustering = DBSCAN(
            eps=self.cluster_eps,
            min_samples=self.cluster_min_samples
        ).fit(points)
        
        labels = clustering.labels_
        unique_labels = set(labels)
        
        clusters = []
        for label in unique_labels:
            if label == -1:  # Noise
                continue
            mask = labels == label
            cluster_points = points[mask]
            if len(cluster_points) >= self.cluster_min_samples:
                clusters.append(cluster_points)
        
        return clusters
    
    def fit_line_ransac(self, points: np.ndarray) -> Optional[LineSegment]:
        """Fit a line segment to points using RANSAC.
        
        Args:
            points: Nx2 array of (x, y) points
        
        Returns:
            LineSegment or None if fitting fails
        """
        if len(points) < self.ransac_min_samples:
            return None
        
        # Determine dominant direction using PCA
        centroid = np.mean(points, axis=0)
        centered = points - centroid
        
        # Covariance and eigenvectors
        cov = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        
        # Principal direction (larger eigenvalue)
        principal_idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, principal_idx]
        
        # Project points onto principal axis
        projections = centered @ direction
        
        # Find inliers (points close to line)
        perpendicular = eigenvectors[:, 1 - principal_idx]
        distances = np.abs(centered @ perpendicular)
        inlier_mask = distances < self.ransac_threshold
        
        if np.sum(inlier_mask) < self.ransac_min_samples:
            return None
        
        # Recompute with inliers
        inlier_points = points[inlier_mask]
        inlier_projections = projections[inlier_mask]
        
        # Segment endpoints from projection extremes
        min_proj = np.min(inlier_projections)
        max_proj = np.max(inlier_projections)
        
        p1 = centroid + min_proj * direction
        p2 = centroid + max_proj * direction
        
        segment = LineSegment(x1=p1[0], y1=p1[1], x2=p2[0], y2=p2[1])
        
        if segment.length < self.min_segment_length:
            return None
        
        return segment
    
    def fit_multiple_lines(self, points: np.ndarray, max_lines: int = 5) -> List[LineSegment]:
        """Iteratively fit multiple lines to a point cluster.
        
        Uses RANSAC repeatedly, removing inliers after each fit.
        """
        segments = []
        remaining = points.copy()
        
        for _ in range(max_lines):
            if len(remaining) < self.ransac_min_samples:
                break
            
            segment = self.fit_line_ransac(remaining)
            if segment is None:
                break
            
            segments.append(segment)
            
            # Remove inliers
            distances = np.array([segment.point_distance(p[0], p[1]) for p in remaining])
            outlier_mask = distances > self.ransac_threshold
            remaining = remaining[outlier_mask]
        
        return segments
    
    def are_collinear(self, seg1: LineSegment, seg2: LineSegment) -> bool:
        """Check if two segments are approximately collinear."""
        # Check angle difference
        angle_diff = abs(seg1.angle - seg2.angle)
        angle_diff = min(angle_diff, math.pi - angle_diff)  # Handle opposite directions
        
        if angle_diff > self.merge_angle_threshold:
            return False
        
        # Check perpendicular distance between midpoints and lines
        mid2 = seg2.midpoint
        dist1 = seg1.point_distance(mid2[0], mid2[1])
        
        mid1 = seg1.midpoint
        dist2 = seg2.point_distance(mid1[0], mid1[1])
        
        return max(dist1, dist2) < self.merge_distance_threshold
    
    def segments_gap(self, seg1: LineSegment, seg2: LineSegment) -> float:
        """Compute the gap distance between two segments along their direction."""
        # Project all endpoints onto seg1's direction
        dx = seg1.x2 - seg1.x1
        dy = seg1.y2 - seg1.y1
        length = seg1.length
        
        if length < 1e-9:
            return float('inf')
        
        dir_x, dir_y = dx / length, dy / length
        
        # Project seg1 endpoints
        proj1_start = (seg1.x1 - seg1.x1) * dir_x + (seg1.y1 - seg1.y1) * dir_y
        proj1_end = (seg1.x2 - seg1.x1) * dir_x + (seg1.y2 - seg1.y1) * dir_y
        
        # Project seg2 endpoints  
        proj2_start = (seg2.x1 - seg1.x1) * dir_x + (seg2.y1 - seg1.y1) * dir_y
        proj2_end = (seg2.x2 - seg1.x1) * dir_x + (seg2.y2 - seg1.y1) * dir_y
        
        # Find ranges
        range1 = (min(proj1_start, proj1_end), max(proj1_start, proj1_end))
        range2 = (min(proj2_start, proj2_end), max(proj2_start, proj2_end))
        
        # Gap is distance between ranges
        if range1[1] < range2[0]:
            return range2[0] - range1[1]
        elif range2[1] < range1[0]:
            return range1[0] - range2[1]
        else:
            return 0  # Overlapping
    
    def merge_segments(self, seg1: LineSegment, seg2: LineSegment) -> LineSegment:
        """Merge two collinear segments into one."""
        # Collect all endpoints
        endpoints = [
            (seg1.x1, seg1.y1),
            (seg1.x2, seg1.y2),
            (seg2.x1, seg2.y1),
            (seg2.x2, seg2.y2)
        ]
        
        # Project onto average direction
        mid = ((seg1.x1 + seg1.x2 + seg2.x1 + seg2.x2) / 4,
               (seg1.y1 + seg1.y2 + seg2.y1 + seg2.y2) / 4)
        
        avg_angle = (seg1.angle + seg2.angle) / 2
        dir_x = math.cos(avg_angle)
        dir_y = math.sin(avg_angle)
        
        projections = [(p[0] * dir_x + p[1] * dir_y, p) for p in endpoints]
        projections.sort(key=lambda x: x[0])
        
        # Take extreme points
        p1 = projections[0][1]
        p2 = projections[-1][1]
        
        return LineSegment(x1=p1[0], y1=p1[1], x2=p2[0], y2=p2[1])
    
    def merge_collinear_segments(self, segments: List[LineSegment]) -> List[LineSegment]:
        """Merge all collinear segments that are close enough."""
        if len(segments) <= 1:
            return segments
        
        merged = list(segments)
        changed = True
        
        while changed:
            changed = False
            new_merged = []
            used = set()
            
            for i, seg1 in enumerate(merged):
                if i in used:
                    continue
                
                current = seg1
                for j, seg2 in enumerate(merged):
                    if j <= i or j in used:
                        continue
                    
                    if self.are_collinear(current, seg2):
                        gap = self.segments_gap(current, seg2)
                        if gap < self.merge_gap_threshold:
                            current = self.merge_segments(current, seg2)
                            used.add(j)
                            changed = True
                
                new_merged.append(current)
                used.add(i)
            
            merged = new_merged
        
        return merged
    
    def snap_endpoints(self, segments: List[LineSegment]) -> List[LineSegment]:
        """Snap nearby endpoints together for cleaner intersections."""
        if len(segments) <= 1:
            return segments
        
        # Collect all endpoints
        endpoints = []
        for i, seg in enumerate(segments):
            endpoints.append((seg.x1, seg.y1, i, 'start'))
            endpoints.append((seg.x2, seg.y2, i, 'end'))
        
        # Find endpoint clusters
        snapped_segments = [
            LineSegment(s.x1, s.y1, s.x2, s.y2) for s in segments
        ]
        
        for i, (x1, y1, seg_i, which1) in enumerate(endpoints):
            for j, (x2, y2, seg_j, which2) in enumerate(endpoints):
                if j <= i or seg_i == seg_j:
                    continue
                
                dist = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if dist < self.snap_distance:
                    # Snap to midpoint
                    mid_x = (x1 + x2) / 2
                    mid_y = (y1 + y2) / 2
                    
                    # Update segments
                    seg = snapped_segments[seg_i]
                    if which1 == 'start':
                        snapped_segments[seg_i] = LineSegment(mid_x, mid_y, seg.x2, seg.y2)
                    else:
                        snapped_segments[seg_i] = LineSegment(seg.x1, seg.y1, mid_x, mid_y)
                    
                    seg = snapped_segments[seg_j]
                    if which2 == 'start':
                        snapped_segments[seg_j] = LineSegment(mid_x, mid_y, seg.x2, seg.y2)
                    else:
                        snapped_segments[seg_j] = LineSegment(seg.x1, seg.y1, mid_x, mid_y)
        
        return snapped_segments
    
    def extract_walls(self, points: np.ndarray) -> List[LineSegment]:
        """Full wall extraction pipeline.
        
        Args:
            points: Nx2 array of merged point cloud
        
        Returns:
            List of wall line segments
        """
        if len(points) < self.cluster_min_samples:
            return []
        
        # Step 1: Cluster points
        clusters = self.cluster_points(points)
        
        # Step 2: Fit lines to each cluster
        all_segments = []
        for cluster in clusters:
            segments = self.fit_multiple_lines(cluster)
            all_segments.extend(segments)
        
        # Step 3: Merge collinear segments
        merged = self.merge_collinear_segments(all_segments)
        
        # Step 4: Snap endpoints
        snapped = self.snap_endpoints(merged)
        
        # Filter by minimum length
        final = [s for s in snapped if s.length >= self.min_segment_length]
        
        return final


def extract_walls(points: np.ndarray, **kwargs) -> List[LineSegment]:
    """Convenience function for wall extraction.
    
    Args:
        points: Nx2 array of (x, y) points
        **kwargs: Parameters for WallExtractor
    
    Returns:
        List of LineSegment objects
    """
    extractor = WallExtractor(**kwargs)
    return extractor.extract_walls(points)
