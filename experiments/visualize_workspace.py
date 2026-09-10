"""Visualize right arm workspace from CSV data."""

import sys
from pathlib import Path
import csv
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read workspace data
csv_path = Path("data/workspace_right_arm.csv")
tcp_positions = []

with open(csv_path, 'r') as f:
    reader = csv.DictReader(f)
    for row in reader:
        tcp_positions.append([
            float(row['tcp_x']),
            float(row['tcp_y']),
            float(row['tcp_z'])
        ])

tcp_positions = np.array(tcp_positions)

# Create 3D scatter plot
fig = plt.figure(figsize=(12, 9))
ax = fig.add_subplot(111, projection='3d')

# Plot workspace
ax.scatter(tcp_positions[:, 0], tcp_positions[:, 1], tcp_positions[:, 2], 
           c=tcp_positions[:, 2], cmap='viridis', s=1, alpha=0.5)

# Plot block position
block_pos = [0.5, 0, 0.65]
ax.scatter(*block_pos, color='red', s=100, marker='X', label='Block (current)')

# Plot better block position (within reach)
# Use the closest reachable point
distances = np.linalg.norm(tcp_positions - np.array([0.5, 0, 0.65]), axis=1)
best_reach = tcp_positions[np.argmin(distances)]
ax.scatter(*best_reach, color='orange', s=100, marker='o', label='Closest reachable')

# Labels and legend
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('Right Arm Workspace')
ax.legend()

# Set equal aspect ratio
all_points = np.vstack([tcp_positions, [block_pos]])
ax.set_xlim(all_points[:, 0].min()-0.1, all_points[:, 0].max()+0.1)
ax.set_ylim(all_points[:, 1].min()-0.1, all_points[:, 1].max()+0.1)
ax.set_zlim(all_points[:, 2].min()-0.1, all_points[:, 2].max()+0.1)

plt.savefig('workspace_visualization.png', dpi=150, bbox_inches='tight')
print("✓ Saved workspace_visualization.png")
plt.show()