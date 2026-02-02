"""Synthetic session generator for Planar.

This module generates synthetic LiDAR and IMU sessions for development,
testing, and CI purposes. The generated data closely mimics real capture
sessions and can be processed by the full pipeline.

Features:
- Configurable room geometry (walls, obstacles)
- Multiple station placements with rotation
- Full 6-axis IMU simulation with realistic noise
- Configurable LiDAR noise and occlusions
- Event timeline generation

Usage:
    python -m simulation.generate_synthetic --out sessions/synthetic --stations 5
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import random
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import List, Tuple, Optional, Dict, Any

import numpy as np


@dataclass
class Wall:
    """A wall segment defined by two endpoints."""
    x1: float
    y1: float
    x2: float
    y2: float
    
    def ray_intersection(self, ox: float, oy: float, dx: float, dy: float) -> Optional[float]:
        """Find intersection distance from ray origin to wall.
        
        Args:
            ox, oy: Ray origin
            dx, dy: Ray direction (normalized)
        
        Returns:
            Distance to intersection, or None if no intersection
        """
        # Wall vector
        wx = self.x2 - self.x1
        wy = self.y2 - self.y1
        
        # Solve: origin + t*dir = wall_start + s*wall_dir
        denom = dx * wy - dy * wx
        
        if abs(denom) < 1e-9:
            return None  # Parallel
        
        t = ((self.x1 - ox) * wy - (self.y1 - oy) * wx) / denom
        s = ((self.x1 - ox) * dy - (self.y1 - oy) * dx) / denom
        
        if t > 0 and 0 <= s <= 1:
            return t
        
        return None


@dataclass
class Room:
    """Room geometry for simulation."""
    walls: List[Wall] = field(default_factory=list)
    
    @classmethod
    def rectangle(cls, width: float = 10.0, height: float = 8.0, center: Tuple[float, float] = (0, 0)) -> 'Room':
        """Create a rectangular room."""
        cx, cy = center
        hw, hh = width / 2, height / 2
        
        walls = [
            Wall(cx - hw, cy - hh, cx + hw, cy - hh),  # Bottom
            Wall(cx + hw, cy - hh, cx + hw, cy + hh),  # Right
            Wall(cx + hw, cy + hh, cx - hw, cy + hh),  # Top
            Wall(cx - hw, cy + hh, cx - hw, cy - hh),  # Left
        ]
        
        return cls(walls=walls)
    
    @classmethod
    def l_shaped(cls, width: float = 10.0, height: float = 8.0) -> 'Room':
        """Create an L-shaped room."""
        walls = [
            Wall(-5, -4, 5, -4),    # Bottom
            Wall(5, -4, 5, 0),      # Right bottom
            Wall(5, 0, 0, 0),       # Inner horizontal
            Wall(0, 0, 0, 4),       # Inner vertical
            Wall(0, 4, -5, 4),      # Top
            Wall(-5, 4, -5, -4),    # Left
        ]
        return cls(walls=walls)
    
    def add_obstacle(self, cx: float, cy: float, size: float = 1.0) -> None:
        """Add a square obstacle (pillar/furniture)."""
        hs = size / 2
        self.walls.extend([
            Wall(cx - hs, cy - hs, cx + hs, cy - hs),
            Wall(cx + hs, cy - hs, cx + hs, cy + hs),
            Wall(cx + hs, cy + hs, cx - hs, cy + hs),
            Wall(cx - hs, cy + hs, cx - hs, cy - hs),
        ])
    
    def cast_ray(self, ox: float, oy: float, angle_rad: float, max_range: float = 50.0) -> float:
        """Cast a ray and return distance to nearest wall."""
        dx = math.cos(angle_rad)
        dy = math.sin(angle_rad)
        
        min_dist = max_range
        
        for wall in self.walls:
            dist = wall.ray_intersection(ox, oy, dx, dy)
            if dist is not None and dist < min_dist:
                min_dist = dist
        
        return min_dist


@dataclass
class Station:
    """Station placement for simulation."""
    x: float
    y: float
    heading: float  # radians, sensor heading in global frame


@dataclass
class LidarConfig:
    """LiDAR simulation parameters."""
    points_per_scan: int = 360
    scan_range_min: float = 0.15  # meters
    scan_range_max: float = 40.0  # meters
    range_noise_stddev: float = 0.01  # meters
    angle_noise_stddev: float = 0.1  # degrees
    dropout_rate: float = 0.02  # probability of missed point


@dataclass
class ImuConfig:
    """IMU simulation parameters."""
    sample_rate: float = 100.0  # Hz
    gyro_noise_stddev: float = 0.001  # rad/s
    gyro_bias: Tuple[float, float, float] = (0.0001, -0.0001, 0.0002)  # rad/s
    accel_noise_stddev: float = 0.01  # m/s²
    gravity: float = 9.81  # m/s²
    temperature: float = 25.0  # °C
    temperature_noise: float = 0.5  # °C


@dataclass
class SyntheticSession:
    """Generator for synthetic sessions."""
    
    room: Room = field(default_factory=Room.rectangle)
    stations: List[Station] = field(default_factory=list)
    lidar_config: LidarConfig = field(default_factory=LidarConfig)
    imu_config: ImuConfig = field(default_factory=ImuConfig)
    
    # Timing
    station_capture_duration: float = 2.0  # seconds per station
    station_move_duration: float = 5.0  # seconds between stations
    
    def place_stations_linear(self, n_stations: int = 3, spacing: float = 2.0) -> None:
        """Place stations in a line along x-axis."""
        self.stations.clear()
        start_x = -spacing * (n_stations - 1) / 2
        
        for i in range(n_stations):
            x = start_x + i * spacing
            # Add small random rotation
            heading = random.gauss(0, math.radians(15))
            self.stations.append(Station(x=x, y=0, heading=heading))
    
    def place_stations_circular(self, n_stations: int = 4, radius: float = 2.0) -> None:
        """Place stations in a circle, facing outward."""
        self.stations.clear()
        
        for i in range(n_stations):
            angle = 2 * math.pi * i / n_stations
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            # Face center
            heading = angle + math.pi + random.gauss(0, math.radians(10))
            self.stations.append(Station(x=x, y=y, heading=heading))
    
    def generate_lidar_scan(self, station: Station) -> List[Dict[str, float]]:
        """Generate a LiDAR scan from a station position."""
        points = []
        cfg = self.lidar_config
        
        for i in range(cfg.points_per_scan):
            # Angle in sensor frame
            sensor_angle = i * 360.0 / cfg.points_per_scan
            
            # Add angle noise
            sensor_angle += random.gauss(0, cfg.angle_noise_stddev)
            
            # Convert to global frame for ray casting
            global_angle = math.radians(sensor_angle) + station.heading
            
            # Cast ray
            distance = self.room.cast_ray(station.x, station.y, global_angle, cfg.scan_range_max)
            
            # Check range validity
            if distance < cfg.scan_range_min or distance > cfg.scan_range_max:
                continue
            
            # Add range noise
            distance += random.gauss(0, cfg.range_noise_stddev)
            
            # Random dropout
            if random.random() < cfg.dropout_rate:
                continue
            
            # Quality decreases with distance
            quality = max(10, min(100, int(150 - distance * 3)))
            
            points.append({
                "angle_deg": sensor_angle % 360,
                "distance_m": max(0, distance),
                "quality": quality
            })
        
        return points
    
    def generate_imu_samples(
        self,
        duration: float,
        start_time: float,
        rotation_rate: float = 0.0,  # rad/s around Z
        tilt: Tuple[float, float] = (0.0, 0.0)  # roll, pitch in radians
    ) -> List[Dict[str, float]]:
        """Generate IMU samples for a time period."""
        samples = []
        cfg = self.imu_config
        
        dt = 1.0 / cfg.sample_rate
        n_samples = int(duration * cfg.sample_rate)
        
        roll, pitch = tilt
        
        for i in range(n_samples):
            t = start_time + i * dt
            
            # Gyroscope (with bias and noise)
            gyro_x = cfg.gyro_bias[0] + random.gauss(0, cfg.gyro_noise_stddev)
            gyro_y = cfg.gyro_bias[1] + random.gauss(0, cfg.gyro_noise_stddev)
            gyro_z = rotation_rate + cfg.gyro_bias[2] + random.gauss(0, cfg.gyro_noise_stddev)
            
            # Accelerometer (gravity + noise, transformed by tilt)
            # For small tilts: accel ≈ [g*sin(pitch), -g*sin(roll), g*cos(roll)*cos(pitch)]
            accel_x = cfg.gravity * math.sin(pitch) + random.gauss(0, cfg.accel_noise_stddev)
            accel_y = -cfg.gravity * math.sin(roll) + random.gauss(0, cfg.accel_noise_stddev)
            accel_z = cfg.gravity * math.cos(roll) * math.cos(pitch) + random.gauss(0, cfg.accel_noise_stddev)
            
            # Temperature
            temp = cfg.temperature + random.gauss(0, cfg.temperature_noise)
            
            samples.append({
                "timestamp": t,
                "gyro_x": gyro_x,
                "gyro_y": gyro_y,
                "gyro_z": gyro_z,
                "accel_x": accel_x,
                "accel_y": accel_y,
                "accel_z": accel_z,
                "temperature": temp
            })
        
        return samples
    
    def generate_session(self, output_dir: Path | str) -> Dict[str, Any]:
        """Generate a complete session.
        
        Args:
            output_dir: Output directory path
        
        Returns:
            Summary dictionary
        """
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        if not self.stations:
            self.place_stations_linear(3)
        
        # Metadata
        metadata = {
            "project": "Planar-synthetic",
            "created": datetime.utcnow().isoformat() + "Z",
            "stations": len(self.stations),
            "room_walls": len(self.room.walls),
            "lidar_config": asdict(self.lidar_config),
            "imu_config": {
                "sample_rate": self.imu_config.sample_rate,
                "gyro_noise_stddev": self.imu_config.gyro_noise_stddev,
                "accel_noise_stddev": self.imu_config.accel_noise_stddev,
            }
        }
        
        with open(output_dir / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)
        
        # Generate data for each station
        events = []
        all_imu_samples = []
        current_time = 0.0
        
        events.append({
            "type": "session_start",
            "timestamp": current_time
        })
        
        for i, station in enumerate(self.stations):
            # Movement phase (except for first station)
            if i > 0:
                prev_station = self.stations[i - 1]
                
                # Calculate rotation during move
                delta_heading = station.heading - prev_station.heading
                rotation_rate = delta_heading / self.station_move_duration
                
                # Generate IMU samples during movement
                move_samples = self.generate_imu_samples(
                    duration=self.station_move_duration,
                    start_time=current_time,
                    rotation_rate=rotation_rate
                )
                all_imu_samples.extend(move_samples)
                current_time += self.station_move_duration
            
            # Capture phase
            events.append({
                "type": "station_captured",
                "station": i,
                "timestamp": current_time,
                "position": {"x": station.x, "y": station.y},
                "heading_deg": math.degrees(station.heading)
            })
            
            # Generate LiDAR scan
            scan_points = self.generate_lidar_scan(station)
            
            # Write station CSV
            station_file = output_dir / f"lidar_station_{i}.csv"
            with open(station_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "angle_deg", "distance_m", "quality"])
                for j, pt in enumerate(scan_points):
                    ts = current_time + j * 0.001  # ~1ms between points
                    writer.writerow([
                        f"{ts:.6f}",
                        f"{pt['angle_deg']:.3f}",
                        f"{pt['distance_m']:.6f}",
                        pt['quality']
                    ])
            
            # Generate IMU samples during capture (stationary)
            capture_samples = self.generate_imu_samples(
                duration=self.station_capture_duration,
                start_time=current_time,
                rotation_rate=0.0  # Stationary
            )
            all_imu_samples.extend(capture_samples)
            current_time += self.station_capture_duration
        
        events.append({
            "type": "session_stop",
            "timestamp": current_time
        })
        
        # Write events
        with open(output_dir / "events.json", "w") as f:
            json.dump(events, f, indent=2)
        
        # Write IMU log
        with open(output_dir / "imu_log.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "gyro_x", "gyro_y", "gyro_z",
                "accel_x", "accel_y", "accel_z", "temperature"
            ])
            for sample in all_imu_samples:
                writer.writerow([
                    f"{sample['timestamp']:.6f}",
                    f"{sample['gyro_x']:.6f}",
                    f"{sample['gyro_y']:.6f}",
                    f"{sample['gyro_z']:.6f}",
                    f"{sample['accel_x']:.6f}",
                    f"{sample['accel_y']:.6f}",
                    f"{sample['accel_z']:.6f}",
                    f"{sample['temperature']:.2f}"
                ])
        
        # Write ground truth poses for validation
        with open(output_dir / "ground_truth.json", "w") as f:
            truth = {
                "stations": [
                    {
                        "index": i,
                        "x": s.x,
                        "y": s.y,
                        "heading_rad": s.heading,
                        "heading_deg": math.degrees(s.heading)
                    }
                    for i, s in enumerate(self.stations)
                ],
                "walls": [
                    {"x1": w.x1, "y1": w.y1, "x2": w.x2, "y2": w.y2}
                    for w in self.room.walls
                ]
            }
            json.dump(truth, f, indent=2)
        
        summary = {
            "status": "ok",
            "session_dir": str(output_dir),
            "stations": len(self.stations),
            "imu_samples": len(all_imu_samples),
            "walls": len(self.room.walls)
        }
        
        print(json.dumps(summary))
        return summary


# Legacy function for backward compatibility
def make_session(outdir: str, n_stations: int = 3, points_per_station: int = 360) -> None:
    """Legacy interface for session generation."""
    session = SyntheticSession()
    session.lidar_config.points_per_scan = points_per_station
    session.place_stations_linear(n_stations)
    session.generate_session(outdir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate synthetic Planar session for testing"
    )
    parser.add_argument(
        "--out",
        required=True,
        help="Output session directory"
    )
    parser.add_argument(
        "--stations",
        type=int,
        default=3,
        help="Number of stations (default: 3)"
    )
    parser.add_argument(
        "--room",
        choices=["rectangle", "l-shaped"],
        default="rectangle",
        help="Room shape (default: rectangle)"
    )
    parser.add_argument(
        "--placement",
        choices=["linear", "circular"],
        default="linear",
        help="Station placement pattern (default: linear)"
    )
    parser.add_argument(
        "--spacing",
        type=float,
        default=2.0,
        help="Station spacing in meters (default: 2.0)"
    )
    parser.add_argument(
        "--noise",
        type=float,
        default=0.01,
        help="LiDAR range noise stddev in meters (default: 0.01)"
    )
    
    args = parser.parse_args()
    
    # Create room
    if args.room == "l-shaped":
        room = Room.l_shaped()
    else:
        room = Room.rectangle()
    
    # Create session generator
    session = SyntheticSession(room=room)
    session.lidar_config.range_noise_stddev = args.noise
    
    # Place stations
    if args.placement == "circular":
        session.place_stations_circular(args.stations, radius=args.spacing)
    else:
        session.place_stations_linear(args.stations, spacing=args.spacing)
    
    # Generate
    session.generate_session(args.out)
