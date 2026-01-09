"""Visualize the generated trajectory to debug letter paths"""
import sys
sys.path.insert(0, 'src')

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from simulator import RobotSimulator

# Create simulator
sim = RobotSimulator()

# Generate trajectory (just waypoints, before IK)
ee_traj = sim.traj_gen.generate_trajectory(sim.config['writing']['letters'])
points_per_segment = sim.config['simulation']['interpolation_points']
interp_traj = sim.traj_gen.interpolate_trajectory(ee_traj, points_per_segment=points_per_segment)

print(f"Generated {len(interp_traj['waypoints'])} waypoints")
print(f"Letter segments: {interp_traj['letter_segments']}")

# Create 3D plot
fig = plt.figure(figsize=(14, 8))
ax = fig.add_subplot(111, projection='3d')

# Draw cardboard
cardboard_pos = sim.config['workspace']['cardboard_position']
cardboard_w = sim.config['workspace']['cardboard_width']
cardboard_h = sim.config['workspace']['cardboard_height']

x_card = [cardboard_pos[0] - cardboard_w/2, cardboard_pos[0] + cardboard_w/2,
          cardboard_pos[0] + cardboard_w/2, cardboard_pos[0] - cardboard_w/2,
          cardboard_pos[0] - cardboard_w/2]
y_card = [cardboard_pos[1]] * 5
z_card = [cardboard_pos[2] - cardboard_h/2, cardboard_pos[2] - cardboard_h/2,
          cardboard_pos[2] + cardboard_h/2, cardboard_pos[2] + cardboard_h/2,
          cardboard_pos[2] - cardboard_h/2]

ax.plot(x_card, y_card, z_card, 'r-', linewidth=3, label='Cardboard (60x120cm)', alpha=0.7)

# Extract trajectory points
waypoints = interp_traj['waypoints']
pen_states = interp_traj['pen_states']

# Separate pen-down and pen-up segments
writing_x, writing_y, writing_z = [], [], []
moving_x, moving_y, moving_z = [], [], []

for i, (wp, pen_down) in enumerate(zip(waypoints, pen_states)):
    if pen_down:
        writing_x.append(wp[0])
        writing_y.append(wp[1])
        writing_z.append(wp[2])
    else:
        moving_x.append(wp[0])
        moving_y.append(wp[1])
        moving_z.append(wp[2])

# Plot trajectories
if writing_x:
    ax.scatter(writing_x, writing_y, writing_z, c='blue', s=20, label='Writing (pen down)', alpha=0.8)
if moving_x:
    ax.scatter(moving_x, moving_y, moving_z, c='green', s=10, label='Moving (pen up)', alpha=0.5)

# Connect waypoints with lines to show the path
for i in range(len(waypoints) - 1):
    wp1 = waypoints[i]
    wp2 = waypoints[i+1]
    color = 'blue' if pen_states[i] else 'lightgray'
    linewidth = 2 if pen_states[i] else 0.5
    ax.plot([wp1[0], wp2[0]], [wp1[1], wp2[1]], [wp1[2], wp2[2]],
            color=color, linewidth=linewidth, alpha=0.6)

# Mark letter segments
colors_seg = ['red', 'orange', 'purple']
for idx, (start, end, letter) in enumerate(interp_traj['letter_segments']):
    if start < len(waypoints):
        wp_start = waypoints[start]
        ax.scatter([wp_start[0]], [wp_start[1]], [wp_start[2]],
                  c=colors_seg[idx % len(colors_seg)], s=100, marker='*',
                  label=f'Letter {letter} start', edgecolors='black', linewidths=1)

# Print statistics
print(f"\nTrajectory Statistics:")
print(f"  X range: {min([w[0] for w in waypoints]):.3f} to {max([w[0] for w in waypoints]):.3f}m")
print(f"  Y range: {min([w[1] for w in waypoints]):.3f} to {max([w[1] for w in waypoints]):.3f}m")
print(f"  Z range: {min([w[2] for w in waypoints]):.3f} to {max([w[2] for w in waypoints]):.3f}m")
print(f"\nCardboard:")
print(f"  Position: {cardboard_pos}")
print(f"  X range: {cardboard_pos[0] - cardboard_w/2:.3f} to {cardboard_pos[0] + cardboard_w/2:.3f}m")
print(f"  Y: {cardboard_pos[1]:.3f}m (constant)")
print(f"  Z range: {cardboard_pos[2] - cardboard_h/2:.3f} to {cardboard_pos[2] + cardboard_h/2:.3f}m")

# Check if trajectory is ON the cardboard surface
y_on_cardboard = all(abs(wp[1] - cardboard_pos[1]) < 0.001 or not pen for wp, pen in zip(waypoints, pen_states))
print(f"\nPen-down points on cardboard Y surface: {y_on_cardboard}")

# Set labels and limits
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('Generated Trajectory Visualization (Before IK)')

ax.set_xlim(-0.8, 0.8)
ax.set_ylim(-0.2, 1.0)
ax.set_zlim(0.0, 1.2)

ax.legend(loc='upper left')

plt.tight_layout()
plt.show()
