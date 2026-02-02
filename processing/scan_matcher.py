"""Scan matching for LiDAR station registration.

This module implements scan-to-scan registration using:
1. Coarse correlative matching (grid-based search)
2. ICP refinement using Open3D

The IMU yaw prior is used to constrain the search space.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Tuple, Optional
import csv

import numpy as np

# Optional Open3D import for ICP
try:
    import open3d as o3d
    HAS_OPEN3D = True
except ImportError:
    o3d = None
    HAS_OPEN3D = False


@dataclass
class ScanPoint:
    """Single LiDAR scan point."""
    x: float
    y: float
    quality: int = 0
    timestamp: float = 0.0


@dataclass
class Pose2D:
    """2D pose (position + orientation)."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # radians, CCW from +X axis
    
    def to_matrix(self) -> np.ndarray:
        """Convert to 3x3 homogeneous transformation matrix."""
        c, s = np.cos(self.theta), np.sin(self.theta)
        return np.array([
            [c, -s, self.x],
            [s,  c, self.y],
            [0,  0, 1]
        ])
    
    @classmethod
    def from_matrix(cls, T: np.ndarray) -> 'Pose2D':
        """Create from 3x3 homogeneous transformation matrix."""
        theta = np.arctan2(T[1, 0], T[0, 0])
        return cls(x=T[0, 2], y=T[1, 2], theta=theta)
    
    def compose(self, other: 'Pose2D') -> 'Pose2D':
        """Compose two poses: self * other."""
        T = self.to_matrix() @ other.to_matrix()
        return Pose2D.from_matrix(T)
    
    def inverse(self) -> 'Pose2D':
        """Return the inverse pose."""
        T_inv = np.linalg.inv(self.to_matrix())
        return Pose2D.from_matrix(T_inv)


@dataclass
class RegistrationResult:
    """Result of scan registration."""
    transform: Pose2D  # Transform from source to target frame
    fitness: float  # Ratio of inlier correspondences
    rmse: float  # Root mean square error of inliers
    converged: bool
    iterations: int = 0
    correspondence_count: int = 0


@dataclass
class ScanMatcher:
    """Scan-to-scan registration using ICP with optional correlative initial guess.
    
    Parameters:
        max_correspondence_distance: Maximum point-to-point distance for correspondence
        fitness_threshold: Minimum fitness score to consider registration valid
        rmse_threshold: Maximum RMSE to consider registration valid
        max_iterations: Maximum ICP iterations
    """
    
    max_correspondence_distance: float = 0.5  # meters
    fitness_threshold: float = 0.3  # minimum 30% correspondences
    rmse_threshold: float = 0.2  # meters
    max_iterations: int = 50
    
    # Correlative search parameters
    search_window_xy: float = 2.0  # meters
    search_window_theta: float = math.pi / 6  # 30 degrees
    search_resolution_xy: float = 0.1  # meters
    search_resolution_theta: float = math.radians(2)  # 2 degrees
    
    def load_station(self, path: Path | str) -> np.ndarray:
        """Load LiDAR station data from CSV.
        
        Returns:
            Nx2 numpy array of (x, y) points
        """
        path = Path(path)
        points = []
        
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames or []
            
            for row in reader:
                # Handle different column formats
                if 'x' in fieldnames and 'y' in fieldnames:
                    # Cartesian format
                    x = float(row['x'])
                    y = float(row['y'])
                elif 'angle_deg' in fieldnames:
                    # Polar format - convert to Cartesian
                    angle_deg = float(row['angle_deg'])
                    distance = float(row.get('distance_m', row.get('range_m', 0)))
                    angle_rad = math.radians(angle_deg)
                    x = distance * math.cos(angle_rad)
                    y = distance * math.sin(angle_rad)
                elif 'angle' in fieldnames:
                    # Old format
                    angle_deg = float(row['angle'])
                    distance = float(row.get('range', row.get('distance', 0)))
                    angle_rad = math.radians(angle_deg)
                    x = distance * math.cos(angle_rad)
                    y = distance * math.sin(angle_rad)
                else:
                    continue
                
                # Filter invalid points
                if math.isfinite(x) and math.isfinite(y):
                    points.append([x, y])
        
        return np.array(points) if points else np.zeros((0, 2))
    
    def transform_points(self, points: np.ndarray, pose: Pose2D) -> np.ndarray:
        """Transform 2D points by a pose.
        
        Args:
            points: Nx2 array of (x, y) points
            pose: Transform to apply
        
        Returns:
            Nx2 array of transformed points
        """
        if len(points) == 0:
            return points
        
        c, s = np.cos(pose.theta), np.sin(pose.theta)
        R = np.array([[c, -s], [s, c]])
        t = np.array([pose.x, pose.y])
        
        return (R @ points.T).T + t
    
    def points_to_pcd(self, points: np.ndarray) -> 'o3d.geometry.PointCloud':
        """Convert 2D points to Open3D point cloud (with z=0)."""
        if not HAS_OPEN3D:
            raise ImportError("Open3D is required for ICP registration")
        
        # Add z=0 for 3D point cloud
        points_3d = np.zeros((len(points), 3))
        points_3d[:, :2] = points
        
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_3d)
        return pcd
    
    def correlative_search(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_guess: Pose2D | None = None,
        yaw_prior: float | None = None,
        yaw_uncertainty: float | None = None
    ) -> Pose2D:
        """Find initial alignment using correlative grid search.
        
        This performs a brute-force search over a discretized pose space,
        using the IMU yaw prior to constrain the rotation search.
        
        Args:
            source: Nx2 source point cloud
            target: Mx2 target point cloud
            initial_guess: Optional initial pose estimate
            yaw_prior: IMU-based yaw change estimate (radians)
            yaw_uncertainty: Uncertainty in yaw prior (radians)
        
        Returns:
            Best pose from grid search
        """
        if initial_guess is None:
            initial_guess = Pose2D()
        
        # Build target KD-tree for fast nearest neighbor queries
        from scipy.spatial import KDTree
        target_tree = KDTree(target)
        
        # Determine search bounds
        theta_center = initial_guess.theta
        theta_range = self.search_window_theta
        
        if yaw_prior is not None:
            theta_center = yaw_prior
            if yaw_uncertainty is not None:
                # Constrain search to 3-sigma around prior
                theta_range = min(self.search_window_theta, 3 * yaw_uncertainty)
        
        # Generate search grid
        x_values = np.arange(
            initial_guess.x - self.search_window_xy,
            initial_guess.x + self.search_window_xy + self.search_resolution_xy,
            self.search_resolution_xy
        )
        y_values = np.arange(
            initial_guess.y - self.search_window_xy,
            initial_guess.y + self.search_window_xy + self.search_resolution_xy,
            self.search_resolution_xy
        )
        theta_values = np.arange(
            theta_center - theta_range,
            theta_center + theta_range + self.search_resolution_theta,
            self.search_resolution_theta
        )
        
        best_score = -float('inf')
        best_pose = initial_guess
        
        # Subsample source for faster search
        step = max(1, len(source) // 100)
        source_subset = source[::step]
        
        for theta in theta_values:
            c, s = np.cos(theta), np.sin(theta)
            R = np.array([[c, -s], [s, c]])
            rotated = (R @ source_subset.T).T
            
            for x in x_values:
                for y in y_values:
                    # Transform points
                    transformed = rotated + np.array([x, y])
                    
                    # Query nearest neighbors
                    distances, _ = target_tree.query(transformed)
                    
                    # Score: count of inliers
                    inliers = np.sum(distances < self.max_correspondence_distance)
                    score = inliers
                    
                    if score > best_score:
                        best_score = score
                        best_pose = Pose2D(x=x, y=y, theta=theta)
        
        return best_pose
    
    def icp_register(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_pose: Pose2D | None = None
    ) -> RegistrationResult:
        """Perform ICP registration using Open3D.
        
        Args:
            source: Nx2 source point cloud
            target: Mx2 target point cloud  
            initial_pose: Initial transformation estimate
        
        Returns:
            RegistrationResult with refined transform
        """
        if not HAS_OPEN3D:
            # Fallback without Open3D
            return self._icp_fallback(source, target, initial_pose)
        
        if initial_pose is None:
            initial_pose = Pose2D()
        
        # Convert to Open3D point clouds
        source_pcd = self.points_to_pcd(source)
        target_pcd = self.points_to_pcd(target)
        
        # Build initial transformation matrix (3D)
        T_init = np.eye(4)
        T_init[:2, :2] = initial_pose.to_matrix()[:2, :2]
        T_init[:2, 3] = [initial_pose.x, initial_pose.y]
        
        # Run ICP
        result = o3d.pipelines.registration.registration_icp(
            source_pcd,
            target_pcd,
            self.max_correspondence_distance,
            T_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.max_iterations
            )
        )
        
        # Extract 2D transform from result
        T = result.transformation
        theta = np.arctan2(T[1, 0], T[0, 0])
        
        return RegistrationResult(
            transform=Pose2D(x=T[0, 3], y=T[1, 3], theta=theta),
            fitness=result.fitness,
            rmse=result.inlier_rmse,
            converged=result.fitness > self.fitness_threshold,
            correspondence_count=len(result.correspondence_set)
        )
    
    def _icp_fallback(
        self,
        source: np.ndarray,
        target: np.ndarray,
        initial_pose: Pose2D | None = None
    ) -> RegistrationResult:
        """Simple ICP implementation when Open3D is not available."""
        from scipy.spatial import KDTree
        
        if initial_pose is None:
            initial_pose = Pose2D()
        
        current_pose = initial_pose
        prev_rmse = float('inf')
        
        for iteration in range(self.max_iterations):
            # Transform source points
            transformed = self.transform_points(source, current_pose)
            
            # Find correspondences
            target_tree = KDTree(target)
            distances, indices = target_tree.query(transformed)
            
            # Filter by distance threshold
            mask = distances < self.max_correspondence_distance
            if np.sum(mask) < 3:
                break
            
            src_matched = transformed[mask]
            tgt_matched = target[indices[mask]]
            
            # Compute RMSE
            rmse = np.sqrt(np.mean(distances[mask]**2))
            
            # Check convergence
            if abs(prev_rmse - rmse) < 1e-6:
                break
            prev_rmse = rmse
            
            # Compute optimal transform (SVD-based)
            src_centroid = np.mean(src_matched, axis=0)
            tgt_centroid = np.mean(tgt_matched, axis=0)
            
            src_centered = src_matched - src_centroid
            tgt_centered = tgt_matched - tgt_centroid
            
            H = src_centered.T @ tgt_centered
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Ensure proper rotation (det = 1)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            t = tgt_centroid - R @ src_centroid
            
            # Update pose
            delta_theta = np.arctan2(R[1, 0], R[0, 0])
            delta_pose = Pose2D(x=t[0], y=t[1], theta=delta_theta)
            current_pose = delta_pose.compose(current_pose)
        
        # Final evaluation
        transformed = self.transform_points(source, current_pose)
        target_tree = KDTree(target)
        distances, _ = target_tree.query(transformed)
        mask = distances < self.max_correspondence_distance
        
        fitness = np.sum(mask) / len(source) if len(source) > 0 else 0
        rmse = np.sqrt(np.mean(distances[mask]**2)) if np.sum(mask) > 0 else float('inf')
        
        return RegistrationResult(
            transform=current_pose,
            fitness=fitness,
            rmse=rmse,
            converged=fitness > self.fitness_threshold and rmse < self.rmse_threshold,
            iterations=iteration + 1,
            correspondence_count=np.sum(mask)
        )
    
    def register(
        self,
        source: np.ndarray,
        target: np.ndarray,
        yaw_prior: float | None = None,
        yaw_uncertainty: float | None = None,
        initial_guess: Pose2D | None = None
    ) -> RegistrationResult:
        """Full registration pipeline: correlative search + ICP refinement.
        
        Args:
            source: Nx2 source point cloud
            target: Mx2 target point cloud
            yaw_prior: IMU-based yaw change estimate (radians)
            yaw_uncertainty: Uncertainty in yaw prior (radians)
            initial_guess: Optional initial pose estimate
        
        Returns:
            RegistrationResult with final transform
        """
        if len(source) < 10 or len(target) < 10:
            return RegistrationResult(
                transform=Pose2D(),
                fitness=0.0,
                rmse=float('inf'),
                converged=False
            )
        
        # Step 1: Correlative search for initial guess
        coarse_pose = self.correlative_search(
            source, target,
            initial_guess=initial_guess,
            yaw_prior=yaw_prior,
            yaw_uncertainty=yaw_uncertainty
        )
        
        # Step 2: ICP refinement
        result = self.icp_register(source, target, initial_pose=coarse_pose)
        
        return result
    
    def validate_registration(self, result: RegistrationResult) -> dict:
        """Validate registration result against quality gates.
        
        Returns:
            Dict with validation status and details
        """
        issues = []
        
        if result.fitness < self.fitness_threshold:
            issues.append(f"Low fitness: {result.fitness:.2f} < {self.fitness_threshold}")
        
        if result.rmse > self.rmse_threshold:
            issues.append(f"High RMSE: {result.rmse:.3f} > {self.rmse_threshold}")
        
        # Sanity checks on transform
        translation_mag = math.sqrt(result.transform.x**2 + result.transform.y**2)
        if translation_mag > 10.0:  # Unlikely for adjacent stations
            issues.append(f"Large translation: {translation_mag:.2f}m")
        
        rotation_deg = abs(math.degrees(result.transform.theta))
        if rotation_deg > 180:
            issues.append(f"Large rotation: {rotation_deg:.1f}°")
        
        return {
            'valid': len(issues) == 0 and result.converged,
            'converged': result.converged,
            'issues': issues,
            'fitness': result.fitness,
            'rmse': result.rmse,
            'translation': translation_mag,
            'rotation_deg': rotation_deg
        }


def register_stations(
    station_paths: list[Path | str],
    yaw_priors: list | None = None
) -> list[Pose2D]:
    """Register multiple stations sequentially.
    
    Args:
        station_paths: Paths to station CSV files
        yaw_priors: Optional list of YawPrior objects
    
    Returns:
        List of poses for each station (first station at origin)
    """
    if len(station_paths) == 0:
        return []
    
    matcher = ScanMatcher()
    
    # Load all stations
    stations = [matcher.load_station(p) for p in station_paths]
    
    # First station is at origin
    poses = [Pose2D()]
    
    # Register each subsequent station to the previous one
    for i in range(1, len(stations)):
        source = stations[i]
        target = stations[i - 1]
        
        # Get yaw prior if available
        yaw_prior = None
        yaw_uncertainty = None
        if yaw_priors and i - 1 < len(yaw_priors):
            prior = yaw_priors[i - 1]
            yaw_prior = prior.delta_yaw_rad
            yaw_uncertainty = prior.uncertainty_rad
        
        result = matcher.register(
            source, target,
            yaw_prior=yaw_prior,
            yaw_uncertainty=yaw_uncertainty
        )
        
        if not result.converged:
            print(f"Warning: Registration failed for station {i}")
        
        # Compose with previous pose
        # transform is source->target, we want global pose of source
        global_pose = poses[-1].compose(result.transform.inverse())
        poses.append(global_pose)
    
    return poses
