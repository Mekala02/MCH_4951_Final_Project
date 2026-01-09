"""Compare generated trajectory waypoints vs IK reached positions"""
import sys
sys.path.insert(0, 'src')

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from simulator import RobotSimulator

# Create simulator
sim = RobotSimulator()

# Generate full trajectory with IK
print("Generating trajectory with IK...")
traj = sim.generate_robot_trajectory()

print(f"\nTotal waypoints: {len(traj['joint_angles'])}")
print(f"IK failures: {(traj['ik_errors'] > 10).sum()}")
print(f"Average IK error: {np.mean(traj['ik_errors'][traj['ik_errors'] < 10]):.6f}m")

# Create 3D comparison plot
fig = plt.figure(figsize=(16, 8))

# Left plot: Generated trajectory
ax1 = fig.add_subplot(121, projection='3d')
ax1.set_title('Generated Trajectory (Target Waypoints)')

# Right plot: IK reached positions
ax2 = fig.add_subplot(122, projection='3d')
ax2.set_title('IK Reached Positions (Actual Robot)')

# Draw cardboard on both
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

for ax in [ax1, ax2]:
    ax.plot(x_card, y_card, z_card, 'r-', linewidth=3, label='Cardboard', alpha=0.7)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_xlim(-0.8, 0.8)
    ax.set_ylim(-0.2, 1.0)
    ax.set_zlim(0.0, 1.2)

# Plot target waypoints (left)
targets = traj['target_positions']
pen_states = traj['pen_states']

target_writing_x, target_writing_y, target_writing_z = [], [], []
target_moving_x, target_moving_y, target_moving_z = [], [], []

for i, (target, pen_down) in enumerate(zip(targets, pen_states)):
    if pen_down:
        target_writing_x.append(target[0])
        target_writing_y.append(target[1])
        target_writing_z.append(target[2])
    else:
        target_moving_x.append(target[0])
        target_moving_y.append(target[1])
        target_moving_z.append(target[2])

if target_writing_x:
    ax1.scatter(target_writing_x, target_writing_y, target_writing_z,
               c='blue', s=20, label='Writing (pen down)', alpha=0.8)
if target_moving_x:
    ax1.scatter(target_moving_x, target_moving_y, target_moving_z,
               c='green', s=10, label='Moving (pen up)', alpha=0.5)

# Plot reached positions (right)
reached = traj['reached_positions']
ik_errors = traj['ik_errors']

reached_writing_x, reached_writing_y, reached_writing_z = [], [], []
reached_moving_x, reached_moving_y, reached_moving_z = [], [], []
failed_x, failed_y, failed_z = [], [], []

for i, (pos, pen_down, error) in enumerate(zip(reached, pen_states, ik_errors)):
    if error > 10:  # IK failure
        failed_x.append(pos[0])
        failed_y.append(pos[1])
        failed_z.append(pos[2])
    elif pen_down:
        reached_writing_x.append(pos[0])
        reached_writing_y.append(pos[1])
        reached_writing_z.append(pos[2])
    else:
        reached_moving_x.append(pos[0])
        reached_moving_y.append(pos[1])
        reached_moving_z.append(pos[2])

if reached_writing_x:
    ax2.scatter(reached_writing_x, reached_writing_y, reached_writing_z,
               c='blue', s=20, label='Writing (pen down)', alpha=0.8)
if reached_moving_x:
    ax2.scatter(reached_moving_x, reached_moving_y, reached_moving_z,
               c='green', s=10, label='Moving (pen up)', alpha=0.5)
if failed_x:
    ax2.scatter(failed_x, failed_y, failed_z,
               c='red', s=50, marker='x', label='IK Failed', alpha=1.0, linewidths=2)

# Connect waypoints with lines
for i in range(len(targets) - 1):
    # Target trajectory
    t1, t2 = targets[i], targets[i+1]
    color = 'blue' if pen_states[i] else 'lightgray'
    linewidth = 2 if pen_states[i] else 0.5
    ax1.plot([t1[0], t2[0]], [t1[1], t2[1]], [t1[2], t2[2]],
            color=color, linewidth=linewidth, alpha=0.6)

    # Reached trajectory (only if both succeeded)
    if ik_errors[i] < 10 and ik_errors[i+1] < 10:
        r1, r2 = reached[i], reached[i+1]
        ax2.plot([r1[0], r2[0]], [r1[1], r2[1]], [r1[2], r2[2]],
                color=color, linewidth=linewidth, alpha=0.6)

ax1.legend(loc='upper left')
ax2.legend(loc='upper left')

plt.tight_layout()

# Print detailed comparison
print("\n=== Detailed Comparison ===")
print(f"Target Y values (should all be {cardboard_pos[1]:.3f}m):")
print(f"  Min: {min([t[1] for t in targets]):.3f}m")
print(f"  Max: {max([t[1] for t in targets]):.3f}m")
print(f"  All at cardboard Y: {all(abs(t[1] - cardboard_pos[1]) < 0.001 for t in targets)}")

print(f"\nReached Y values:")
print(f"  Min: {min([r[1] for r in reached]):.3f}m")
print(f"  Max: {max([r[1] for r in reached]):.3f}m")
all_at_y = all(abs(r[1] - cardboard_pos[1]) < 0.001 for r in reached)
print(f"  All at cardboard Y: {all_at_y}")

print(f"\nTarget Z range: {min([t[2] for t in targets]):.3f} to {max([t[2] for t in targets]):.3f}m")
print(f"Reached Z range: {min([r[2] for r in reached]):.3f} to {max([r[2] for r in reached]):.3f}m")

# Find worst IK errors (excluding failures)
successful_errors = [(i, err) for i, err in enumerate(ik_errors) if err < 10]
successful_errors.sort(key=lambda x: x[1], reverse=True)

print(f"\nWorst 5 IK errors (successful):")
for i, err in successful_errors[:5]:
    target = targets[i]
    reached_pos = reached[i]
    print(f"  Waypoint {i}: error={err:.6f}m")
    print(f"    Target:  [{target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f}]")
    print(f"    Reached: [{reached_pos[0]:.3f}, {reached_pos[1]:.3f}, {reached_pos[2]:.3f}]")

plt.show()
