"""IMU processing for yaw prior computation.

This module integrates gyroscope readings to compute yaw (heading) changes
between LiDAR stations. The yaw priors help constrain the scan registration
problem and improve convergence.
"""
from __future__ import annotations

import csv
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterator


@dataclass
class ImuSample:
    """Single IMU sample."""
    timestamp: float
    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0  # rad/s, positive = CCW when viewed from above
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    temperature: float = 25.0


@dataclass
class YawPrior:
    """Yaw change between two stations."""
    from_station: int
    to_station: int
    delta_yaw_rad: float  # Integrated yaw change
    uncertainty_rad: float  # Estimated uncertainty
    sample_count: int  # Number of IMU samples integrated
    duration_sec: float  # Time span of integration


@dataclass 
class ImuProcessor:
    """Process IMU data to extract yaw priors for scan registration.
    
    The processor integrates gyro Z readings between station capture events
    to compute the expected rotation between consecutive scans. This provides
    a prior for the ICP registration, improving convergence and robustness.
    
    Drift Correction:
    - Assumes stationary periods during station captures
    - Uses stationary samples to estimate gyro bias
    - Subtracts bias from integration
    
    Uncertainty Estimation:
    - Based on integration duration and sample rate
    - Accounts for gyro noise and bias instability
    """
    
    # Gyro noise parameters (typical for BMI160)
    gyro_noise_density: float = 0.007  # rad/s/√Hz (typical BMI160)
    gyro_bias_instability: float = 0.0003  # rad/s (typical BMI160)
    
    # Stationary detection threshold
    stationary_threshold: float = 0.02  # rad/s - below this, consider stationary
    
    # Minimum samples for bias estimation
    min_bias_samples: int = 50
    
    _samples: list[ImuSample] = field(default_factory=list)
    _station_events: list[dict] = field(default_factory=list)
    
    def load_imu_log(self, path: Path | str) -> None:
        """Load IMU samples from CSV file.
        
        Supports both simple format (timestamp, gyro_z_rad_s) and 
        full format (timestamp, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, temperature).
        """
        path = Path(path)
        self._samples.clear()
        
        with open(path, newline='') as f:
            reader = csv.DictReader(f)
            fieldnames = reader.fieldnames or []
            
            for row in reader:
                sample = ImuSample(timestamp=float(row['timestamp']))
                
                # Handle different column naming conventions
                if 'gyro_z_rad_s' in fieldnames:
                    # Simple format from simulator
                    sample.gyro_z = float(row['gyro_z_rad_s'])
                elif 'gyro_z' in fieldnames:
                    # Full format from capture daemon
                    sample.gyro_x = float(row.get('gyro_x', 0))
                    sample.gyro_y = float(row.get('gyro_y', 0))
                    sample.gyro_z = float(row.get('gyro_z', 0))
                    sample.accel_x = float(row.get('accel_x', 0))
                    sample.accel_y = float(row.get('accel_y', 0))
                    sample.accel_z = float(row.get('accel_z', 0))
                    sample.temperature = float(row.get('temperature', 25))
                
                self._samples.append(sample)
        
        # Sort by timestamp
        self._samples.sort(key=lambda s: s.timestamp)
    
    def load_events(self, path: Path | str) -> None:
        """Load station events from JSON file."""
        import json
        path = Path(path)
        
        with open(path) as f:
            events = json.load(f)
        
        # Filter to station capture events
        self._station_events = [
            e for e in events 
            if e.get('type') == 'station_captured'
        ]
        
        # Sort by timestamp
        self._station_events.sort(key=lambda e: e['timestamp'])
    
    def get_samples_in_range(self, t_start: float, t_end: float) -> list[ImuSample]:
        """Get all IMU samples within a time range."""
        return [s for s in self._samples if t_start <= s.timestamp <= t_end]
    
    def estimate_bias(self, samples: list[ImuSample]) -> tuple[float, float, float]:
        """Estimate gyro bias from stationary samples.
        
        Returns:
            Tuple of (bias_x, bias_y, bias_z) in rad/s
        """
        if len(samples) < self.min_bias_samples:
            return 0.0, 0.0, 0.0
        
        # Find stationary periods (low gyro magnitude)
        stationary_samples = []
        for s in samples:
            mag = math.sqrt(s.gyro_x**2 + s.gyro_y**2 + s.gyro_z**2)
            if mag < self.stationary_threshold:
                stationary_samples.append(s)
        
        if len(stationary_samples) < self.min_bias_samples:
            # Fall back to using all samples if not enough stationary
            stationary_samples = samples
        
        # Compute mean as bias estimate
        n = len(stationary_samples)
        bias_x = sum(s.gyro_x for s in stationary_samples) / n
        bias_y = sum(s.gyro_y for s in stationary_samples) / n
        bias_z = sum(s.gyro_z for s in stationary_samples) / n
        
        return bias_x, bias_y, bias_z
    
    def integrate_yaw(
        self, 
        samples: list[ImuSample], 
        bias_z: float = 0.0
    ) -> tuple[float, float]:
        """Integrate gyro Z to compute yaw change.
        
        Args:
            samples: IMU samples to integrate
            bias_z: Gyro Z bias to subtract (rad/s)
        
        Returns:
            Tuple of (delta_yaw_rad, uncertainty_rad)
        """
        if len(samples) < 2:
            return 0.0, float('inf')
        
        delta_yaw = 0.0
        
        for i in range(1, len(samples)):
            dt = samples[i].timestamp - samples[i-1].timestamp
            gyro_z_corrected = samples[i].gyro_z - bias_z
            delta_yaw += gyro_z_corrected * dt
        
        # Estimate uncertainty based on integration
        duration = samples[-1].timestamp - samples[0].timestamp
        sample_rate = len(samples) / max(duration, 1e-6)
        
        # Random walk uncertainty: σ = noise_density * √(duration)
        random_walk = self.gyro_noise_density * math.sqrt(duration)
        
        # Bias uncertainty: grows linearly with time
        bias_drift = self.gyro_bias_instability * duration
        
        # Combined uncertainty (RSS)
        uncertainty = math.sqrt(random_walk**2 + bias_drift**2)
        
        return delta_yaw, uncertainty
    
    def compute_yaw_priors(self) -> list[YawPrior]:
        """Compute yaw priors between all consecutive stations.
        
        Returns:
            List of YawPrior objects for each station transition
        """
        if len(self._station_events) < 2:
            return []
        
        priors = []
        
        # First, estimate global bias from all samples
        all_bias = self.estimate_bias(self._samples)
        
        for i in range(len(self._station_events) - 1):
            event_from = self._station_events[i]
            event_to = self._station_events[i + 1]
            
            t_start = event_from['timestamp']
            t_end = event_to['timestamp']
            
            # Get samples between stations
            samples = self.get_samples_in_range(t_start, t_end)
            
            if len(samples) < 2:
                # No IMU data for this transition
                priors.append(YawPrior(
                    from_station=event_from.get('station', i),
                    to_station=event_to.get('station', i + 1),
                    delta_yaw_rad=0.0,
                    uncertainty_rad=math.pi,  # Maximum uncertainty
                    sample_count=0,
                    duration_sec=t_end - t_start
                ))
                continue
            
            # Try to estimate local bias from samples around capture events
            # Use samples near the start (likely stationary during capture)
            local_bias = self.estimate_bias(samples[:min(100, len(samples))])
            
            # Use local bias if available, otherwise fall back to global
            bias_z = local_bias[2] if any(local_bias) else all_bias[2]
            
            delta_yaw, uncertainty = self.integrate_yaw(samples, bias_z)
            
            priors.append(YawPrior(
                from_station=event_from.get('station', i),
                to_station=event_to.get('station', i + 1),
                delta_yaw_rad=delta_yaw,
                uncertainty_rad=uncertainty,
                sample_count=len(samples),
                duration_sec=t_end - t_start
            ))
        
        return priors
    
    def check_level(self, samples: list[ImuSample] | None = None) -> dict:
        """Check if the device is level using accelerometer data.
        
        Returns:
            Dict with tilt angles and level status
        """
        if samples is None:
            samples = self._samples
        
        if not samples:
            return {'level': None, 'error': 'No samples'}
        
        # Get samples with accelerometer data
        accel_samples = [s for s in samples if s.accel_z != 0]
        if not accel_samples:
            return {'level': None, 'error': 'No accelerometer data'}
        
        # Average accelerometer readings
        n = len(accel_samples)
        avg_x = sum(s.accel_x for s in accel_samples) / n
        avg_y = sum(s.accel_y for s in accel_samples) / n
        avg_z = sum(s.accel_z for s in accel_samples) / n
        
        # Compute tilt angles
        # Roll (rotation about X axis) = atan2(accel_y, accel_z)
        # Pitch (rotation about Y axis) = atan2(-accel_x, sqrt(accel_y^2 + accel_z^2))
        roll_rad = math.atan2(avg_y, avg_z)
        pitch_rad = math.atan2(-avg_x, math.sqrt(avg_y**2 + avg_z**2))
        
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)
        
        # Consider level if within ±2 degrees
        is_level = abs(roll_deg) < 2.0 and abs(pitch_deg) < 2.0
        
        return {
            'level': is_level,
            'roll_deg': roll_deg,
            'pitch_deg': pitch_deg,
            'gravity_magnitude': math.sqrt(avg_x**2 + avg_y**2 + avg_z**2)
        }


def load_yaw_priors(session_dir: Path | str) -> list[YawPrior]:
    """Convenience function to load and compute yaw priors from a session.
    
    Args:
        session_dir: Path to session directory
    
    Returns:
        List of YawPrior objects
    """
    session_dir = Path(session_dir)
    
    processor = ImuProcessor()
    
    imu_log = session_dir / 'imu_log.csv'
    events_file = session_dir / 'events.json'
    
    if imu_log.exists():
        processor.load_imu_log(imu_log)
    
    if events_file.exists():
        processor.load_events(events_file)
    
    return processor.compute_yaw_priors()
