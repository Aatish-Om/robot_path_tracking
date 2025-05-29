"""
path_plotter.py

This script visualizes and compares a raw waypoint path with a smoothed path
using matplotlib. It uses the `smooth_path` function to generate the smooth path
and saves the resulting plot as an image.

Author: Aatish Om
"""

import os
import matplotlib.pyplot as plt
from turtlebot3_path_tracking.path_tracker.path_smoother import smooth_path

# Create the output directory if it doesn't exist
os.makedirs('plots', exist_ok=True)

# Original waypoints (example path)
waypoints = [(0, 0), (1, 0), (2, 2), (3, 0), (4, 2)]

# Generate the smoothed path using cubic spline interpolation
smoothed_path = smooth_path(waypoints)

# Separate x and y coordinates for plotting
wx, wy = zip(*waypoints)
sx, sy = zip(*smoothed_path)

# Create the plot
plt.figure(figsize=(8, 6))
plt.plot(wx, wy, 'ro--', label='Original Waypoints')
plt.plot(sx, sy, 'b-', label='Smoothed Path')

# Add labels, grid, and legend
plt.title('Path Comparison: Waypoints vs Smoothed Path')
plt.xlabel('X Position (m)')
plt.ylabel('Y Position (m)')
plt.legend()
plt.grid(True)

# Save plot to file and display it
plt.savefig('plots/path_comparison.png')
plt.show()

