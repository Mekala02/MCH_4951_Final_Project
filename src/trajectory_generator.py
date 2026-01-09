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
                (0.0, 0.0),
                (0.0, 1.0),
                (0.6, 1.0),
                None,  # Pen retract
                (0.0, 0.5),
                (0.5, 0.5),
                None,  # Pen retract
                (0.0, 0.0),
                (0.6, 0.0)
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
            ],
            'G': [
                (0.6, 0.8),
                (0.5, 1.0),
                (0.1, 1.0),
                (0.0, 0.9),
                (0.0, 0.1),
                (0.1, 0.0),
                (0.5, 0.0),
                (0.6, 0.1),
                (0.6, 0.5),
                (0.3, 0.5)
            ],
            'Z': [
                (0.0, 1.0),
                (0.6, 1.0),
                (0.0, 0.0),
                (0.6, 0.0)
            ],
            'M': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.3, 0.5),
                (0.6, 1.0),
                (0.6, 0.0)
            ],
            'C': [
                (0.6, 0.2),
                (0.5, 0.0),
                (0.1, 0.0),
                (0.0, 0.1),
                (0.0, 0.9),
                (0.1, 1.0),
                (0.5, 1.0),
                (0.6, 0.8)
            ],
            'D': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.4, 1.0),
                (0.6, 0.8),
                (0.6, 0.2),
                (0.4, 0.0),
                (0.0, 0.0)
            ],
            'F': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.6, 1.0),
                (0.6, 1.0),
                (0.0, 0.5),
                (0.5, 0.5)
            ],
            'J': [
                (0.6, 1.0),
                (0.6, 0.2),
                (0.5, 0.0),
                (0.1, 0.0),
                (0.0, 0.1)
            ],
            'K': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.0, 0.5),
                (0.6, 1.0),
                (0.0, 0.5),
                (0.6, 0.0)
            ],
            'O': [
                (0.1, 0.0),
                (0.0, 0.1),
                (0.0, 0.9),
                (0.1, 1.0),
                (0.5, 1.0),
                (0.6, 0.9),
                (0.6, 0.1),
                (0.5, 0.0),
                (0.1, 0.0)
            ],
            'P': [
                (0.0, 0.0),
                (0.0, 1.0),
                (0.5, 1.0),
                (0.6, 0.9),
                (0.6, 0.6),
                (0.5, 0.5),
                (0.0, 0.5)
            ],
            'Q': [
                (0.1, 0.0),
                (0.0, 0.1),
                (0.0, 0.9),
                (0.1, 1.0),
                (0.5, 1.0),
                (0.6, 0.9),
                (0.6, 0.1),
                (0.5, 0.0),
                (0.1, 0.0),
                (0.1, 0.0),
                (0.4, 0.3),
                (0.6, 0.0)
            ],
            'S': [
                (0.6, 0.8),
                (0.5, 1.0),
                (0.1, 1.0),
                (0.0, 0.8),
                (0.1, 0.6),
                (0.5, 0.4),
                (0.6, 0.2),
                (0.5, 0.0),
                (0.1, 0.0),
                (0.0, 0.2)
            ],
            'V': [
                (0.0, 1.0),
                (0.3, 0.0),
                (0.6, 1.0)
            ],
            'W': [
                (0.0, 1.0),
                (0.15, 0.0),
                (0.3, 0.5),
                (0.45, 0.0),
                (0.6, 1.0)
            ],
            'X': [
                (0.0, 0.0),
                (0.6, 1.0),
                (0.6, 1.0),
                (0.3, 0.5),
                (0.3, 0.5),
                (0.0, 1.0),
                (0.6, 0.0)
            ],
            'Y': [
                (0.0, 1.0),
                (0.3, 0.5),
                (0.6, 1.0),
                (0.3, 0.5),
                (0.3, 0.0)
            ],
            'HEART': [
                (0.3, 0.5),
                (0.15, 0.7),
                (0.0, 0.9),
                (0.0, 1.0),
                (0.15, 1.0),
                (0.3, 0.85),
                (0.45, 1.0),
                (0.6, 1.0),
                (0.6, 0.9),
                (0.45, 0.7),
                (0.3, 0.5),
                (0.3, 0.0)
            ]
        }
        return letters

    def scale_letter(self, waypoints, letter_index, num_letters):
        """
        Scale normalized letter waypoints to cardboard coordinates

        Args:
            waypoints: List of (x, y) normalized coordinates or None for pen retractions
            letter_index: 0, 1, or 2 for letter position
            num_letters: Total number of letters to center them

        Returns:
            List of (x, y, z) 3D coordinates or None for pen retractions
        """
        scaled_points = []

        # Calculate letter width from config
        letter_width_ratio = self.config['writing']['letter_width_ratio']
        letter_width = self.letter_height * letter_width_ratio

        # Get margins from config
        v_margin = self.config['writing']['vertical_margin']

        # Calculate total width of all letters plus spacing
        total_letters_width = num_letters * letter_width + (num_letters - 1) * self.letter_spacing

        # Center the letters on the cardboard horizontally
        # Start from center, subtract half the total width
        start_x = self.cardboard_pos[0] - total_letters_width / 2
        start_x += letter_index * (letter_width + self.letter_spacing)

        # Y is constant (distance from robot to cardboard)
        y_pos = self.cardboard_pos[1]

        # Z is vertical (height on cardboard) - center vertically
        base_z = self.cardboard_pos[2] - self.letter_height / 2

        for waypoint in waypoints:
            if waypoint is None:
                # Pen retraction marker
                scaled_points.append(None)
            else:
                norm_x, norm_y = waypoint
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
            letters: String of letters to write (can include special token "HEART")

        Returns:
            dict with:
                - waypoints: List of (x, y, z) positions for each letter
                - letter_segments: List of (start_idx, end_idx, letter) tuples
        """
        letter_paths = self.get_letter_paths()
        all_waypoints = []
        letter_segments = []

        # Parse letters, handling "HEART" as a special token
        parsed_letters = []
        i = 0
        letters_upper = letters.upper()
        while i < len(letters_upper):
            if letters_upper[i:i+5] == 'HEART':
                parsed_letters.append('HEART')
                i += 5
            else:
                parsed_letters.append(letters_upper[i])
                i += 1

        # Calculate number of valid letters for centering
        num_letters = len(parsed_letters)

        for letter_idx, letter in enumerate(parsed_letters):
            if letter not in letter_paths:
                print(f"Warning: Letter '{letter}' not defined, skipping")
                continue

            # Get scaled waypoints for this letter
            letter_waypoints = self.scale_letter(letter_paths[letter], letter_idx, num_letters)

            # If not the first letter, add travel waypoints from previous letter
            if letter_idx > 0:
                # Previous letter should have ended with a retraction
                # Now travel to the start of this letter (while retracted)
                first_point = next(p for p in letter_waypoints if p is not None)
                travel_to_next = first_point.copy()
                travel_to_next[1] -= self.retract_dist  # Stay retracted
                all_waypoints.append(travel_to_next)

                # Approach the cardboard for the new letter
                all_waypoints.append(first_point)

            segment_start = len(all_waypoints)

            # Add all letter waypoints, handling None markers for pen retractions
            last_valid_point = None
            for point in letter_waypoints:
                if point is None:
                    # Pen retraction marker: retract from last point, then approach next point
                    if last_valid_point is not None:
                        # Retract from current position
                        retract_point = last_valid_point.copy()
                        retract_point[1] -= self.retract_dist
                        all_waypoints.append(retract_point)
                else:
                    # Check if we need to approach after a retraction
                    if last_valid_point is not None and all_waypoints[-1][1] < last_valid_point[1]:
                        # We're retracted, need to travel and approach
                        travel_point = point.copy()
                        travel_point[1] -= self.retract_dist
                        all_waypoints.append(travel_point)
                        # Approach the cardboard
                        all_waypoints.append(point)
                    else:
                        # Normal waypoint
                        all_waypoints.append(point)
                    last_valid_point = point

            segment_end = len(all_waypoints)
            letter_segments.append((segment_start, segment_end, letter))

            # After finishing this letter, retract if not the last letter
            if letter_idx < len(parsed_letters) - 1:
                # Retract at end of this letter (pull pen away from cardboard)
                last_point = all_waypoints[-1].copy()
                retract_point = last_point.copy()
                retract_point[1] -= self.retract_dist  # Pull back away from cardboard
                all_waypoints.append(retract_point)

        return {
            'waypoints': np.array(all_waypoints),
            'letter_segments': letter_segments
        }

    def interpolate_trajectory(self, trajectory, points_per_segment=20):
        """
        Interpolate trajectory including ALL waypoints (letters AND retraction/travel)

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

        # Track which waypoints belong to letter segments
        letter_waypoint_indices = set()
        for start_idx, end_idx, letter in letter_segments:
            for i in range(start_idx, end_idx):
                letter_waypoint_indices.add(i)

        # Interpolate ALL waypoints sequentially (including retraction/travel)
        current_letter_idx = 0
        in_letter = False
        letter_interp_start = 0

        for i in range(len(waypoints) - 1):
            start_pos = waypoints[i]
            end_pos = waypoints[i + 1]

            # Track when we enter/exit letter segments
            if i in letter_waypoint_indices and not in_letter:
                # Starting a new letter
                in_letter = True
                letter_interp_start = len(all_interpolated_points)
            elif i not in letter_waypoint_indices and in_letter:
                # Just finished a letter
                in_letter = False
                if current_letter_idx < len(letter_segments):
                    _, _, letter_char = letter_segments[current_letter_idx]
                    interpolated_letter_segments.append((letter_interp_start, len(all_interpolated_points), letter_char))
                    current_letter_idx += 1

            # Create SE3 transforms
            T_start = SE3(start_pos[0], start_pos[1], start_pos[2])
            T_end = SE3(end_pos[0], end_pos[1], end_pos[2])

            # Use RTB's ctraj for smooth interpolation
            traj = rtb.ctraj(T_start, T_end, t=points_per_segment)

            # Extract positions (skip last point to avoid duplicates)
            for T in traj[:-1]:
                all_interpolated_points.append(T.t)

        # Add final waypoint
        all_interpolated_points.append(waypoints[-1])

        # If we're still in a letter at the end, close it
        if in_letter and current_letter_idx < len(letter_segments):
            _, _, letter_char = letter_segments[current_letter_idx]
            interpolated_letter_segments.append((letter_interp_start, len(all_interpolated_points), letter_char))

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
