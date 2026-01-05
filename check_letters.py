"""Check where each letter is positioned"""

import yaml
from trajectory_generator import TrajectoryGenerator

with open('config.yaml', 'r') as f:
    config = yaml.safe_load(f)

traj_gen = TrajectoryGenerator(config)
trajectory = traj_gen.generate_trajectory('BAU')

print(f"Cardboard position: {config['workspace']['cardboard_position']}")
print(f"Cardboard size: {config['workspace']['cardboard_width']}m x {config['workspace']['cardboard_height']}m")
print(f"Letter height: {config['writing']['letter_height']}m")
print(f"Letter spacing: {config['writing']['letter_spacing']}m\n")

# Check each letter's bounds
for start, end, letter in trajectory['letter_segments']:
    waypoints = trajectory['waypoints'][start:end]

    x_vals = [w[0] for w in waypoints]
    y_vals = [w[1] for w in waypoints]
    z_vals = [w[2] for w in waypoints]

    print(f"Letter '{letter}':")
    print(f"  X range: {min(x_vals):.3f} to {max(x_vals):.3f}")
    print(f"  Y range: {min(y_vals):.3f} to {max(y_vals):.3f}")
    print(f"  Z range: {min(z_vals):.3f} to {max(z_vals):.3f}")

# Check cardboard bounds
cardboard_pos = config['workspace']['cardboard_position']
cardboard_w = config['workspace']['cardboard_width']
cardboard_h = config['workspace']['cardboard_height']

print(f"\nCardboard bounds:")
print(f"  X: {cardboard_pos[0] - cardboard_w/2:.3f} to {cardboard_pos[0] + cardboard_w/2:.3f}")
print(f"  Y: {cardboard_pos[1]:.3f} (flat surface)")
print(f"  Z: {cardboard_pos[2] - cardboard_h/2:.3f} to {cardboard_pos[2] + cardboard_h/2:.3f}")
