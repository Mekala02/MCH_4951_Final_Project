"""
Swift-based Simulator - Uses RTB's web viewer for proper URDF visualization
Shows robot with correct offset geometry automatically
"""

import numpy as np
import yaml
import os
import time
from roboticstoolbox.robot.Robot import Robot

import sys
sys.path.append(os.path.dirname(__file__))

# Apply Swift patch for Python 3.12 BEFORE importing swift
from swift_patch import patch_swift_for_python312
patch_swift_for_python312()

import swift
from trajectory_generator import TrajectoryGenerator


class SwiftSimulator:
    """Simulator using Swift web viewer"""

    def __init__(self, config_path=None):
        """Initialize simulator"""
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        with open(config_path, 'r', encoding='utf-8') as f:
            self.config = yaml.safe_load(f)

        # Load robot from URDF
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'robot.urdf')
        self.robot = Robot.URDF(urdf_path)
        self.robot.gravity = [0, 0, -self.config['simulation']['gravity']]

        # Set all geometry colors to white (URDF materials not parsed correctly by RTB)
        for link in self.robot.links:
            if link.geometry:
                for geom in link.geometry:
                    geom.color = (1.0, 1.0, 1.0, 1.0)  # White RGBA

        self.traj_gen = TrajectoryGenerator(self.config)
        self.trajectory = None

    def generate_trajectory(self, letters=None):
        """Generate trajectory and solve IK"""
        if letters is None:
            letters = self.config['writing']['letters']

        print(f"Generating trajectory for '{letters}'...")

        # Generate waypoints
        traj_data = self.traj_gen.generate_trajectory(letters)

        # Interpolate trajectory to add intermediate points
        points_per_segment = self.config['simulation']['interpolation_points']
        interp_traj = self.traj_gen.interpolate_trajectory(traj_data, points_per_segment=points_per_segment)

        waypoints = interp_traj['waypoints']
        letter_segments = interp_traj['letter_segments']

        # Solve IK for all waypoints
        print(f"Solving IK for {len(waypoints)} waypoints...")

        joint_angles = []
        ik_errors = []

        # Initial guess - use zeros for all joints (generic for any DOF)
        n_joints = self.robot.n
        q_prev = np.zeros(n_joints)
        # Set reasonable starting position for prismatic joint if it exists
        for i, link in enumerate(self.robot.links):
            if i > 0 and hasattr(link, 'isprismatic') and link.isprismatic:
                q_prev[i-1] = 0.12  # Mid-range for prismatic joint (link i connects joint i-1)

        progress_interval = self.config['simulation'].get('progress_report_interval', 50)

        for i, target_pos in enumerate(waypoints):
            if i % progress_interval == 0:
                avg_error = np.mean(ik_errors) if ik_errors else 0.0
                print(f"  Progress: {i}/{len(waypoints)}, IK error: {avg_error:.4f}m")

            # Target transform (position only, no orientation constraint)
            from spatialmath import SE3
            T_target = SE3(target_pos)

            # Solve IK
            mask = self.config['ik']['position_mask']
            sol = self.robot.ikine_LM(T_target, q0=q_prev, mask=mask)

            if sol.success:
                q = sol.q
                # Verify solution
                T_actual = self.robot.fkine(q)
                error = np.linalg.norm(T_actual.t - target_pos)

                # Limit joint step size for smoothness
                max_step = self.config['ik'].get('max_joint_step', 0.8)
                q_diff = q - q_prev
                if np.max(np.abs(q_diff)) > max_step:
                    q = q_prev + q_diff * (max_step / np.max(np.abs(q_diff)))

                q_prev = q
                ik_errors.append(error)
            else:
                print(f"  Warning: IK failed at waypoint {i}: {sol.reason}")
                q = q_prev
                ik_errors.append(999.0)

            joint_angles.append(q)

        self.trajectory = {
            'waypoints': waypoints,
            'joint_angles': np.array(joint_angles),
            'letter_segments': letter_segments,
            'ik_errors': ik_errors
        }

        # Report statistics
        print()
        print("=== Trajectory Analysis ===")
        avg_error = np.mean(ik_errors)
        max_error = np.max(ik_errors)
        print(f"Average IK error: {avg_error:.4f}m")
        print(f"Max IK error: {max_error:.4f}m")
        print()

        return self.trajectory

    def animate_swift(self, speed=1.0):
        """Animate trajectory using Swift web viewer"""
        if self.trajectory is None:
            print("No trajectory generated. Call generate_trajectory() first.")
            return

        print("Launching Swift web viewer...")
        print("This will open in your browser with proper URDF geometry!")
        print()

        # Create Swift environment - this starts background threads
        env = swift.Swift()

        # Launch with realtime=True for smooth animation
        env.launch(realtime=True)

        # Give Swift time to initialize
        time.sleep(2)

        # Add robot to Swift scene
        env.add(self.robot, collision_alpha=False)

        # Add cardboard surface visualization
        from spatialgeometry import Box
        from spatialmath import SE3
        cardboard_width = self.config['workspace']['cardboard_width']
        cardboard_height = self.config['workspace']['cardboard_height']
        cardboard_pos = np.array(self.config['workspace']['cardboard_position'])

        # Cardboard is VERTICAL in XZ plane
        # cardboard_pos is where letters should be drawn (front surface facing robot)
        # Move cardboard FORWARD by half thickness so front surface is at cardboard_pos
        # Letters appear on the side facing the robot (smaller Y = closer to robot)
        # X = horizontal, Y = depth (distance from robot), Z = vertical height
        # Box dimensions: [width_X, depth_Y, height_Z]
        cardboard_thickness = 0.01
        cardboard_center = cardboard_pos.copy()
        cardboard_center[1] += cardboard_thickness / 2  # Move forward in +Y direction (away from robot)

        cardboard = Box(
            scale=[cardboard_width, cardboard_thickness, cardboard_height],  # Wide in X, thin in Y, tall in Z
            pose=SE3(cardboard_center),  # Position at center (shifted forward)
            color=[0.9, 0.85, 0.7]  # Tan/cardboard color
        )
        env.add(cardboard)
        print(f"Cardboard front surface (facing robot) at Y={cardboard_pos[1]}, center at {cardboard_center}")

        print("Swift launched! Animating trajectory...")
        print("Press Ctrl+C to stop.")
        print()

        joint_angles = self.trajectory['joint_angles']
        letter_segments = self.trajectory['letter_segments']

        # Interpolate between waypoints for smooth motion
        from scipy.interpolate import interp1d

        # Create time array for waypoints
        n_waypoints = len(joint_angles)
        t_waypoints = np.linspace(0, 1, n_waypoints)

        # Interpolate each joint
        interpolators = []
        for joint_idx in range(joint_angles.shape[1]):
            interp = interp1d(t_waypoints, joint_angles[:, joint_idx], kind='cubic')
            interpolators.append(interp)

        # Generate smooth trajectory with many interpolated points
        n_frames = n_waypoints * 20  # 20 frames between each waypoint
        t_frames = np.linspace(0, 1, n_frames)

        smooth_trajectory = np.zeros((n_frames, joint_angles.shape[1]))
        for joint_idx, interp in enumerate(interpolators):
            smooth_trajectory[:, joint_idx] = interp(t_frames)

        dt = 0.015  # 10ms per frame (2x faster)

        # First, draw the TARGET trajectory (desired letter path) in GREEN
        from spatialgeometry import Sphere, Box
        from spatialmath import SE3
        waypoints = self.trajectory['waypoints']

        print("Drawing target trajectory (red for letters, green for retractions)...")

        # Determine cardboard Y position for distinguishing letter strokes from retractions
        cardboard_y = cardboard_pos[1]
        retract_threshold = cardboard_y - 0.02  # 2cm tolerance

        # Draw ALL waypoint connections with color coding
        red_count = 0
        green_count = 0

        for i in range(1, len(waypoints)):
            prev_wp = waypoints[i-1]
            wp = waypoints[i]

            direction = wp - prev_wp
            length = np.linalg.norm(direction)

            if length > 0.001:
                midpoint = (prev_wp + wp) / 2

                # Determine color based on Y position (distance from robot)
                # If both points are close to cardboard Y, it's a letter stroke (RED)
                # If either point is retracted (smaller Y), it's a travel/retraction (GREEN)
                avg_y = (prev_wp[1] + wp[1]) / 2
                if avg_y > retract_threshold:
                    color = [1, 0, 0]  # Red for letter strokes
                    red_count += 1
                else:
                    color = [0, 1, 0]  # Green for retractions/travel
                    green_count += 1

                # Calculate rotation to align box with line direction
                # Box default orientation: length along X-axis
                x_axis = np.array([1, 0, 0])
                direction_norm = direction / length

                # Create rotation from x-axis to direction
                if np.allclose(direction_norm, x_axis):
                    R = np.eye(3)
                elif np.allclose(direction_norm, -x_axis):
                    R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
                else:
                    # Rodrigues rotation formula
                    axis = np.cross(x_axis, direction_norm)
                    axis_len = np.linalg.norm(axis)
                    if axis_len > 0.0001:
                        axis = axis / axis_len
                        angle = np.arccos(np.clip(np.dot(x_axis, direction_norm), -1, 1))
                        K = np.array([[0, -axis[2], axis[1]],
                                     [axis[2], 0, -axis[0]],
                                     [-axis[1], axis[0], 0]])
                        R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
                    else:
                        R = np.eye(3)

                # Create transform
                T = SE3.Rt(R, midpoint)

                # Thin box line with color based on type
                box = Box(scale=[length, 0.004, 0.004], pose=T, color=color)
                env.add(box)

        print(f"  Drew {red_count} red segments (letters) and {green_count} green segments (retractions)")

        print("Target trajectory drawn. Starting robot animation...")

        # Track end effector path for visualization
        ee_path_points = []

        try:
            for i, q in enumerate(smooth_trajectory):
                # Update robot configuration
                self.robot.q = q

                # Get current end effector position
                T_ee = self.robot.fkine(q)
                ee_pos = T_ee.t
                ee_path_points.append(ee_pos)

                # Draw trajectory line segments as we animate (blue lines showing robot path)
                if i > 0 and i % 5 == 0:  # Draw every 5th frame to reduce clutter
                    from spatialgeometry import Cylinder
                    from spatialmath import SE3
                    prev_pos = ee_path_points[i-1]

                    # Calculate cylinder parameters
                    direction = ee_pos - prev_pos
                    length = np.linalg.norm(direction)

                    if length > 0.001:  # Only draw if points are far enough apart
                        midpoint = (prev_pos + ee_pos) / 2

                        # Calculate orientation to align cylinder with direction
                        z_axis = np.array([0, 0, 1])
                        direction_norm = direction / length

                        # Create rotation from z-axis to direction
                        if np.allclose(direction_norm, z_axis):
                            T = SE3(midpoint)
                        elif np.allclose(direction_norm, -z_axis):
                            T = SE3.Rx(np.pi) * SE3(midpoint)
                        else:
                            # Rodrigues rotation
                            axis = np.cross(z_axis, direction_norm)
                            axis = axis / np.linalg.norm(axis)
                            angle = np.arccos(np.dot(z_axis, direction_norm))
                            T = SE3(midpoint) * SE3.AngleAxis(angle, axis)

                        # Create thin blue cylinder for the trajectory
                        cyl = Cylinder(radius=0.002, length=length, pose=T, color=[0, 0, 1])
                        env.add(cyl)

                # Step simulation with actual time delay
                env.step(dt / speed)
                time.sleep(dt / speed)

                if i % 50 == 0:
                    print(f"  Frame {i}/{n_frames}")

            print()
            print("Animation complete!")
            print(f"Total trajectory points drawn: {len(ee_path_points)}")
            print()
            print("Visualization legend:")
            print("  RED = Target trajectory (desired letter path)")
            print("  BLUE = Actual trajectory (where robot actually went)")
            print()
            print("If blue matches red, IK succeeded. If they diverge, IK failed.")
            print()
            print("Press Ctrl+C to exit...")

            # Hold window open - wait for user interrupt
            try:
                while True:
                    env.step(0.1)
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("\nExiting...")

        except KeyboardInterrupt:
            print("\nAnimation stopped by user")


def main():
    """Main entry point"""
    try:
        sim = SwiftSimulator()

        # Generate trajectory
        sim.generate_trajectory()

        # Animate using Swift (speed multiplier: higher = faster)
        sim.animate_swift(speed=2.0)

    except KeyboardInterrupt:
        print("\nStopped by user")


if __name__ == '__main__':
    main()
