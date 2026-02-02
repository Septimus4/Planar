"""Pose graph optimization for global consistency.

This module implements pose graph SLAM for optimizing the relative poses
between LiDAR stations after pairwise registration. This ensures global
consistency when there are loop closures or accumulated drift.

Uses scipy.optimize for the optimization backend.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Tuple, Optional, Callable
import numpy as np

try:
    from scipy.optimize import minimize, least_squares
    from scipy.sparse import lil_matrix
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

from .scan_matcher import Pose2D, RegistrationResult


@dataclass
class PoseGraphEdge:
    """Edge in the pose graph representing a constraint between two nodes."""
    from_node: int
    to_node: int
    measurement: Pose2D  # Relative transform from -> to
    information: np.ndarray = field(default_factory=lambda: np.eye(3))  # 3x3 info matrix
    
    @classmethod
    def from_registration(
        cls,
        from_node: int,
        to_node: int,
        result: RegistrationResult,
        yaw_uncertainty: float = 0.1
    ) -> 'PoseGraphEdge':
        """Create edge from registration result.
        
        Information matrix is derived from registration quality.
        """
        # Build information matrix from fitness and RMSE
        # Higher fitness and lower RMSE = more confident
        trans_info = result.fitness / max(result.rmse, 0.01)
        rot_info = result.fitness / max(yaw_uncertainty, 0.01)
        
        information = np.diag([trans_info, trans_info, rot_info])
        
        return cls(
            from_node=from_node,
            to_node=to_node,
            measurement=result.transform,
            information=information
        )


@dataclass
class PoseGraph:
    """Pose graph for multi-station optimization.
    
    Nodes are poses (stations), edges are relative pose constraints from:
    - Sequential registration (station i to station i-1)
    - IMU yaw priors
    - Loop closures (if detected)
    """
    
    nodes: list[Pose2D] = field(default_factory=list)
    edges: list[PoseGraphEdge] = field(default_factory=list)
    fixed_nodes: set[int] = field(default_factory=lambda: {0})  # First node fixed
    
    def add_node(self, pose: Pose2D) -> int:
        """Add a node and return its index."""
        idx = len(self.nodes)
        self.nodes.append(pose)
        return idx
    
    def add_edge(self, edge: PoseGraphEdge) -> None:
        """Add an edge constraint."""
        self.edges.append(edge)
    
    def add_sequential_edge(
        self,
        from_node: int,
        to_node: int,
        transform: Pose2D,
        trans_stddev: float = 0.1,
        rot_stddev: float = 0.05
    ) -> None:
        """Add edge from sequential registration."""
        information = np.diag([
            1.0 / (trans_stddev ** 2),
            1.0 / (trans_stddev ** 2),
            1.0 / (rot_stddev ** 2)
        ])
        
        self.edges.append(PoseGraphEdge(
            from_node=from_node,
            to_node=to_node,
            measurement=transform,
            information=information
        ))
    
    def add_imu_prior_edge(
        self,
        from_node: int,
        to_node: int,
        delta_yaw: float,
        yaw_stddev: float
    ) -> None:
        """Add edge from IMU yaw prior (rotation only constraint)."""
        # Create edge with very low translation information (unknown)
        # and IMU-derived rotation information
        information = np.diag([
            0.01,  # Low translation confidence
            0.01,
            1.0 / (yaw_stddev ** 2)  # High rotation confidence
        ])
        
        self.edges.append(PoseGraphEdge(
            from_node=from_node,
            to_node=to_node,
            measurement=Pose2D(x=0, y=0, theta=delta_yaw),
            information=information
        ))
    
    def _pose_to_vector(self, pose: Pose2D) -> np.ndarray:
        """Convert pose to parameter vector [x, y, theta]."""
        return np.array([pose.x, pose.y, pose.theta])
    
    def _vector_to_pose(self, v: np.ndarray) -> Pose2D:
        """Convert parameter vector to pose."""
        return Pose2D(x=v[0], y=v[1], theta=v[2])
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def compute_error(self, edge: PoseGraphEdge, poses: list[Pose2D]) -> np.ndarray:
        """Compute error vector for an edge.
        
        Error = measurement ⊖ (inverse(pose_from) ⊕ pose_to)
        """
        pose_from = poses[edge.from_node]
        pose_to = poses[edge.to_node]
        
        # Compute relative pose: T_from^-1 * T_to
        relative = pose_from.inverse().compose(pose_to)
        
        # Error in measurement frame
        # e = T_measurement^-1 * T_relative
        error_pose = edge.measurement.inverse().compose(relative)
        
        # Return as vector with normalized angle
        error = np.array([
            error_pose.x,
            error_pose.y,
            self._normalize_angle(error_pose.theta)
        ])
        
        return error
    
    def compute_total_error(self, x: np.ndarray) -> float:
        """Compute total weighted squared error for all edges.
        
        Args:
            x: Flattened parameter vector [x0,y0,θ0, x1,y1,θ1, ...]
        """
        # Reconstruct poses
        poses = []
        for i in range(len(self.nodes)):
            if i in self.fixed_nodes:
                poses.append(self.nodes[i])
            else:
                idx = (i - len([n for n in self.fixed_nodes if n < i])) * 3
                poses.append(self._vector_to_pose(x[idx:idx+3]))
        
        total = 0.0
        for edge in self.edges:
            error = self.compute_error(edge, poses)
            total += error.T @ edge.information @ error
        
        return total
    
    def compute_residuals(self, x: np.ndarray) -> np.ndarray:
        """Compute residual vector for least squares.
        
        Args:
            x: Flattened parameter vector for non-fixed nodes
        
        Returns:
            Weighted residual vector
        """
        # Reconstruct all poses
        poses = []
        var_idx = 0
        for i in range(len(self.nodes)):
            if i in self.fixed_nodes:
                poses.append(self.nodes[i])
            else:
                poses.append(self._vector_to_pose(x[var_idx*3:(var_idx+1)*3]))
                var_idx += 1
        
        # Compute weighted residuals for each edge
        residuals = []
        for edge in self.edges:
            error = self.compute_error(edge, poses)
            # Weight by sqrt of information (Cholesky)
            L = np.linalg.cholesky(edge.information)
            weighted_error = L @ error
            residuals.extend(weighted_error)
        
        return np.array(residuals)
    
    def optimize(self, max_iterations: int = 100, tolerance: float = 1e-6) -> dict:
        """Optimize the pose graph using Levenberg-Marquardt.
        
        Returns:
            Dict with optimization results
        """
        if not HAS_SCIPY:
            return {'success': False, 'error': 'scipy not available'}
        
        if len(self.nodes) == 0:
            return {'success': True, 'iterations': 0, 'poses': []}
        
        if len(self.edges) == 0:
            return {'success': True, 'iterations': 0, 'poses': self.nodes.copy()}
        
        # Build initial parameter vector (non-fixed nodes only)
        x0 = []
        for i, pose in enumerate(self.nodes):
            if i not in self.fixed_nodes:
                x0.extend([pose.x, pose.y, pose.theta])
        x0 = np.array(x0)
        
        if len(x0) == 0:
            # All nodes fixed
            return {'success': True, 'iterations': 0, 'poses': self.nodes.copy()}
        
        # Run optimization
        result = least_squares(
            self.compute_residuals,
            x0,
            method='lm',
            max_nfev=max_iterations,
            ftol=tolerance,
            xtol=tolerance
        )
        
        # Extract optimized poses
        optimized_poses = []
        var_idx = 0
        for i in range(len(self.nodes)):
            if i in self.fixed_nodes:
                optimized_poses.append(self.nodes[i])
            else:
                optimized_poses.append(self._vector_to_pose(
                    result.x[var_idx*3:(var_idx+1)*3]
                ))
                var_idx += 1
        
        # Update internal state
        self.nodes = optimized_poses
        
        return {
            'success': result.success,
            'iterations': result.nfev,
            'cost': result.cost,
            'poses': optimized_poses,
            'message': result.message
        }
    
    def get_global_poses(self) -> list[Pose2D]:
        """Get the current global poses of all nodes."""
        return self.nodes.copy()


def build_pose_graph(
    initial_poses: list[Pose2D],
    registration_results: list[RegistrationResult],
    yaw_priors: list = None
) -> PoseGraph:
    """Build a pose graph from sequential registration results.
    
    Args:
        initial_poses: Initial pose estimates for each station
        registration_results: Pairwise registration results (i to i-1)
        yaw_priors: Optional IMU yaw priors
    
    Returns:
        Configured PoseGraph ready for optimization
    """
    graph = PoseGraph()
    
    # Add nodes
    for pose in initial_poses:
        graph.add_node(pose)
    
    # Add sequential edges from registration
    for i, result in enumerate(registration_results):
        edge = PoseGraphEdge.from_registration(
            from_node=i + 1,
            to_node=i,
            result=result,
            yaw_uncertainty=0.1  # Default
        )
        graph.add_edge(edge)
    
    # Add IMU prior edges if available
    if yaw_priors:
        for i, prior in enumerate(yaw_priors):
            if i + 1 < len(initial_poses):
                graph.add_imu_prior_edge(
                    from_node=i,
                    to_node=i + 1,
                    delta_yaw=prior.delta_yaw_rad,
                    yaw_stddev=prior.uncertainty_rad
                )
    
    return graph
