"""Trajectory logger for simulation data."""

import json
from pathlib import Path
from datetime import datetime
import numpy as np


class TrajectoryLogger:
    """Log simulation trajectories to disk."""
    
    def __init__(self, output_dir: str = "data"):
        """Initialize logger."""
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)
        
        self.trajectory = []
        self.start_time = datetime.now()
    
    def record(self, observation: dict):
        """Record a single timestep."""
        # Convert numpy arrays to lists for JSON serialization
        record = {
            "time": float(observation["time"]),
            "joint_angles": observation["joint_angles"].tolist(),
            "joint_velocities": observation["joint_velocities"].tolist(),
            "block_position": observation["block_position"].tolist(),
            "block_velocity": observation["block_velocity"].tolist(),
        }
        self.trajectory.append(record)
    
    def save(self, filename: str = None):
        """Save trajectory to file."""
        if filename is None:
            timestamp = self.start_time.strftime("%Y%m%d_%H%M%S")
            filename = f"trajectory_{timestamp}.json"
        
        filepath = self.output_dir / filename
        
        with open(filepath, 'w') as f:
            json.dump(self.trajectory, f, indent=2)
        
        print(f"✓ Saved trajectory: {filepath} ({len(self.trajectory)} steps)")
        return filepath