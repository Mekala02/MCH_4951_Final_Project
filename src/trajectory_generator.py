"""
Trajectory Generator - Letter Path Generation
Creates waypoints for writing letters on cardboard
"""

import numpy as np


class TrajectoryGenerator:
    """Generate trajectories for writing letters"""

    def __init__(self, config):
        """
        Initialize trajectory generator

        Args:
            config: Robot configuration dictionary
        """
        self.config = config
        self.cardboard_pos = np.array(config['workspace']['cardboard_position'])
        self.cardboard_w = config['workspace']['cardboard_width']
        self.cardboard_h = config['workspace']['cardboard_height']
        self.letter_height = config['writing']['letter_height']
        self.letter_spacing = config['writing']['letter_spacing']
        self.retract_dist = config['writing']['pen_retract_distance']

    def get_letter_paths(self):
        """
        Generate normalized paths for each letter (0-1 scale)

        Returns:
            dict: Letter name -> list of (x, y) waypoints
        """
        letters = {
            'B': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.5, 1.0),
                (0.6, 0.9),
                (0.6, 0.6),
                (0.5, 0.5),
                (0.0, 0.5),
                (0.5, 0.5),
                (0.6, 0.4),
                (0.6, 0.1),
                (0.5, 0.0),
                (0.0, 0.0)
            ],
            'A': [
                (0.0, 0.0),
                (0.3, 1.0),
                (0.6, 0.0),
                (0.6, 0.0),  # Duplicate to lift pen
                (0.15, 0.4),
                (0.45, 0.4)
            ],
            'U': [
                (0.0, 1.0),
                (0.0, 0.2),
                (0.1, 0.05),
                (0.3, 0.0),
                (0.5, 0.05),
                (0.6, 0.2),
                (0.6, 1.0)
            ],
            'I': [
                (0.0, 0.0),
                (0.5, 0.0),
                (0.25, 0.0),
                (0.25, 1.0),
                (0.0, 1.0),
                (0.5, 1.0)
            ],
            'L': [
                (0.0, 1.0),
                (0.0, 0.0),
                (0.6, 0.0)
            ],
            'T': [
                (0.0, 1.0),
                (0.6, 1.0),
                (0.3, 1.0),
                (0.3, 0.0)
            ],
            'E': [
                (0.6, 0.0),
                (0.0, 0.0),
                (0.0, 1.0),
                (0.6, 1.0),
                (0.6, 1.0),  # Duplicate
                (0.0, 0.5),
                (0.5, 0.5)
            ],
            'H': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 0.5),
                (0.6, 0.5),
                (0.6, 0.0),
                (0.6, 1.0)
            ],
            'R': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.5, 1.0),
                (0.6, 0.9),
                (0.6, 0.6),
                (0.5, 0.5),
                (0.0, 0.5),
                (0.6, 0.0)  # Diagonal leg
            ],
            'N': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.6, 0.0),
                (0.6, 1.0)
            ]
        }
        return letters

    def scale_letter(self, waypoints, letter_index):
        """
        Scale normalized letter waypoints to cardboard coordinates

        Args:
            waypoints: List of (x, y) normalized coordinates
            letter_index: 0, 1, or 2 for letter position

        Returns:
            List of (x, y, z) 3D coordinates
        """
        scaled_points = []

        # Calculate letter width from config
        letter_width_ratio = self.config['writing']['letter_width_ratio']
        letter_width = self.letter_height * letter_width_ratio

        # Get margins from config
        h_margin = self.config['writing']['horizontal_margin']
        v_margin = self.config['writing']['vertical_margin']

        # Cardboard is vertical in XZ plane (Y is constant, facing the cardboard)
        # Starting X position for this letter (horizontal on cardboard)
        start_x = self.cardboard_pos[0] - self.cardboard_w/2 + h_margin
        start_x += letter_index * (letter_width + self.letter_spacing)

        # Y is constant (distance from robot to cardboard)
        y_pos = self.cardboard_pos[1]

        # Z is vertical (height on cardboard)
        base_z = self.cardboard_pos[2] - self.cardboard_h/2 + v_margin

        for (norm_x, norm_y) in waypoints:
            # Map normalized coordinates to cardboard surface
            # norm_x -> X (horizontal across cardboard)
            # norm_y -> Z (vertical up cardboard)
            x = start_x + norm_x * letter_width
            y = y_pos  # Fixed distance to cardboard
            z = base_z + norm_y * self.letter_height  # Vertical position

            scaled_points.append([x, y, z])

        return scaled_points

    def generate_trajectory(self, letters):
        """
        Generate complete trajectory for writing letters
        Includes travel waypoints to prevent going through cardboard

        Args:
            letters: String of letters to write

        Returns:
            dict with:
                - waypoints: List of (x, y, z) positions for each letter
                - letter_segments: List of (start_idx, end_idx, letter) tuples
        """
        letter_paths = self.get_letter_paths()
        all_waypoints = []
        letter_segments = []

        for letter_idx, letter in enumerate(letters.upper()):
            if letter not in letter_paths:
                print(f"Warning: Letter '{letter}' not defined, skipping")
                continue

            # Get scaled waypoints for this letter
            letter_waypoints = self.scale_letter(letter_paths[letter], letter_idx)

            segment_start = len(all_waypoints)

            # If not the first letter, add travel waypoints from previous letter
            if letter_idx > 0:
                # Get the last point of previous letter
                prev_last_point = all_waypoints[-1].copy()

                # Retract from previous letter (pull back in -Y direction)
                retract_from_prev = prev_last_point.copy()
                retract_from_prev[1] -= self.retract_dist  # Pull back away from cardboard
                all_waypoints.append(retract_from_prev)

                # Travel to next letter position (while retracted)
                travel_to_next = letter_waypoints[0].copy()
                travel_to_next[1] -= self.retract_dist  # Stay retracted
                all_waypoints.append(travel_to_next)

                # Approach the cardboard for the new letter
                all_waypoints.append(letter_waypoints[0])

                # Add remaining letter waypoints (skip first since we added it as approach)
                for point in letter_waypoints[1:]:
                    all_waypoints.append(point)
            else:
                # First letter - add all waypoints
                for point in letter_waypoints:
                    all_waypoints.append(point)

            segment_end = len(all_waypoints)
            letter_segments.append((segment_start, segment_end, letter))

        return {
            'waypoints': np.array(all_waypoints),
            'letter_segments': letter_segments
        }

    def interpolate_trajectory(self, trajectory, points_per_segment=20):
        """
        Interpolate trajectory letter-by-letter
        SIMPLIFIED: Interpolate each letter separately

        Args:
            trajectory: Output from generate_trajectory()
            points_per_segment: Number of interpolation points between waypoints

        Returns:
            dict with interpolated waypoints for each letter
        """
        from spatialmath import SE3
        import roboticstoolbox as rtb

        waypoints = trajectory['waypoints']
        letter_segments = trajectory['letter_segments']

        all_interpolated_points = []
        interpolated_letter_segments = []

        # Interpolate each letter separately
        for start_idx, end_idx, letter in letter_segments:
            letter_waypoints = waypoints[start_idx:end_idx]

            interp_start = len(all_interpolated_points)
            letter_interpolated = []

            # Interpolate between consecutive waypoints within this letter
            for i in range(len(letter_waypoints) - 1):
                start_pos = letter_waypoints[i]
                end_pos = letter_waypoints[i + 1]

                # Create SE3 transforms
                T_start = SE3(start_pos[0], start_pos[1], start_pos[2])
                T_end = SE3(end_pos[0], end_pos[1], end_pos[2])

                # Use RTB's ctraj for smooth interpolation
                traj = rtb.ctraj(T_start, T_end, t=points_per_segment)

                # Extract positions
                for T in traj:
                    letter_interpolated.append(T.t)

            # Add final waypoint of the letter
            letter_interpolated.append(letter_waypoints[-1])

            all_interpolated_points.extend(letter_interpolated)
            interp_end = len(all_interpolated_points)
            interpolated_letter_segments.append((interp_start, interp_end, letter))

        return {
            'waypoints': np.array(all_interpolated_points),
            'letter_segments': interpolated_letter_segments
        }


if __name__ == '__main__':
    # Test trajectory generation
    import yaml
    import os

    config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)

    traj_gen = TrajectoryGenerator(config)

    # Generate trajectory for "BAU"
    letters = config['writing']['letters']
    trajectory = traj_gen.generate_trajectory(letters)

    print(f"Generated trajectory for '{letters}'")
    print(f"Total waypoints: {len(trajectory['waypoints'])}")
    print(f"Letter segments: {len(trajectory['letter_segments'])}")

    for start, end, letter in trajectory['letter_segments']:
        print(f"  Letter '{letter}': waypoints {start} to {end}")

    # Test interpolation
    interp_traj = traj_gen.interpolate_trajectory(trajectory, points_per_segment=10)
    print(f"\nInterpolated trajectory: {len(interp_traj['waypoints'])} points")

    # Print first few waypoints
    print("\nFirst 5 waypoints:")
    for i in range(min(5, len(trajectory['waypoints']))):
        wp = trajectory['waypoints'][i]
        pen = "DOWN" if trajectory['pen_states'][i] else "UP"
        print(f"  {i}: [{wp[0]:.3f}, {wp[1]:.3f}, {wp[2]:.3f}] - Pen {pen}")
