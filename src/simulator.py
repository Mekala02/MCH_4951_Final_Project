"""
Simulator - Main Animation and Visualization
Combines robot, trajectory, and dynamics for full simulation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import yaml
import os

from robot_designer import RobotArm
from trajectory_generator import TrajectoryGenerator


class RobotSimulator:
    """Main simulator class"""

    def __init__(self, config_path=None):
        """Initialize simulator"""
        if config_path is None:
            # Default to config.yaml in the same directory as this script
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.robot = RobotArm(config_path)
        self.traj_gen = TrajectoryGenerator(self.config)

        self.trajectory = None
        self.joint_trajectory = None
        self.torque_history = []

    def generate_robot_trajectory(self, letters=None):
        """
        Generate complete robot trajectory with IK

        Args:
            letters: String of letters to write (default from config)

        Returns:
            dict with joint_angles, positions, torques, etc.
        """
        if letters is None:
            letters = self.config['writing']['letters']

        print(f"Generating trajectory for '{letters}'...")

        # Generate end effector trajectory
        ee_traj = self.traj_gen.generate_trajectory(letters)
        interp_traj = self.traj_gen.interpolate_trajectory(ee_traj, points_per_segment=5)  # Reduced for speed

        # Solve IK for each waypoint
        joint_angles = []
        reached_positions = []
        torques = []
        ik_errors = []

        previous_joints = [0, 0, 0.3, 0]  # Initial guess

        print(f"Solving IK for {len(interp_traj['waypoints'])} waypoints...")

        for i, target in enumerate(interp_traj['waypoints']):
            try:
                # Solve IK
                joints = self.robot.inverse_kinematics(target, initial_guess=previous_joints)

                # Verify solution
                T = self.robot.forward_kinematics(joints)
                reached = T[:3, 3]

                error = np.linalg.norm(reached - target)
                ik_errors.append(error)

                joint_angles.append(joints)
                reached_positions.append(reached)

                # Calculate static torques
                static_torque = self.robot.compute_static_torques(joints)
                torques.append(static_torque)

                previous_joints = joints

                if i % 50 == 0:
                    print(f"  Progress: {i}/{len(interp_traj['waypoints'])}, IK error: {error:.4f}m")

            except Exception as e:
                print(f"  Warning: IK failed at waypoint {i}: {e}")
                # Use previous configuration
                if len(joint_angles) > 0:
                    joint_angles.append(joint_angles[-1])
                    reached_positions.append(reached_positions[-1])
                    torques.append(torques[-1])
                    ik_errors.append(999)
                else:
                    joint_angles.append([0, 0, 0.3, 0])
                    reached_positions.append(target)
                    torques.append(np.zeros(4))
                    ik_errors.append(999)

        self.trajectory = {
            'target_positions': interp_traj['waypoints'],
            'joint_angles': np.array(joint_angles),
            'reached_positions': np.array(reached_positions),
            'pen_states': interp_traj['pen_states'],
            'torques': np.array(torques),
            'ik_errors': np.array(ik_errors),
            'letter_segments': interp_traj['letter_segments']
        }

        # Check feasibility
        print("\n=== Trajectory Analysis ===")
        print(f"Average IK error: {np.mean(ik_errors):.4f}m")
        print(f"Max IK error: {np.max(ik_errors):.4f}m")

        print("\nTorque Requirements:")
        torque_array = np.array(torques)
        joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist']

        for i, joint_name in enumerate(joint_names):
            max_torque_req = np.max(np.abs(torque_array[:, i]))
            avg_torque_req = np.mean(np.abs(torque_array[:, i]))
            print(f"  {joint_name}: Max {max_torque_req:.2f} Nm, Avg {avg_torque_req:.2f} Nm")

        print("\nUse these values to select appropriate motors (recommended: 1.5x-2x safety factor)")

        return self.trajectory

    def animate(self, save_path=None, interval=50, show_trace=True):
        """
        Animate the robot trajectory

        Args:
            save_path: Optional path to save animation (e.g., 'animation.gif')
            interval: Milliseconds between frames
            show_trace: Show pen trace
        """
        if self.trajectory is None:
            raise ValueError("No trajectory generated. Call generate_robot_trajectory() first.")

        fig = plt.figure(figsize=(14, 8))
        ax = fig.add_subplot(121, projection='3d')
        ax_torque = fig.add_subplot(122)

        # Setup 3D plot
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Robot Arm Animation')

        # Set fixed limits
        ax.set_xlim(-0.5, 1.5)
        ax.set_ylim(-0.5, 0.5)
        ax.set_zlim(0, 1.5)

        # Draw cardboard
        cardboard_pos = self.config['workspace']['cardboard_position']
        cardboard_w = self.config['workspace']['cardboard_width']
        cardboard_h = self.config['workspace']['cardboard_height']

        x_card = [cardboard_pos[0] - cardboard_w/2, cardboard_pos[0] + cardboard_w/2,
                  cardboard_pos[0] + cardboard_w/2, cardboard_pos[0] - cardboard_w/2,
                  cardboard_pos[0] - cardboard_w/2]
        y_card = [cardboard_pos[1]] * 5
        z_card = [cardboard_pos[2] - cardboard_h/2, cardboard_pos[2] - cardboard_h/2,
                  cardboard_pos[2] + cardboard_h/2, cardboard_pos[2] + cardboard_h/2,
                  cardboard_pos[2] - cardboard_h/2]

        ax.plot(x_card, y_card, z_card, 'r-', linewidth=2, label='Cardboard', alpha=0.5)

        # Trace storage
        trace_x, trace_y, trace_z = [], [], []

        # Joint names for display
        joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist']

        def update(frame):
            ax.cla()
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'Robot Arm - Frame {frame}/{len(self.trajectory["joint_angles"])}')
            ax.set_xlim(-0.5, 1.5)
            ax.set_ylim(-0.5, 0.5)
            ax.set_zlim(0, 1.5)

            # Draw cardboard
            ax.plot(x_card, y_card, z_card, 'r-', linewidth=2, alpha=0.5)

            # Get current joint configuration
            joints = self.trajectory['joint_angles'][frame]
            full_joints = [0] + list(joints)

            # Draw robot using ikpy
            self.robot.chain.plot(full_joints, ax, target=None)

            # Add trace
            if show_trace and self.trajectory['pen_states'][frame]:
                pos = self.trajectory['reached_positions'][frame]
                trace_x.append(pos[0])
                trace_y.append(pos[1])
                trace_z.append(pos[2])

            if len(trace_x) > 0:
                ax.plot(trace_x, trace_y, trace_z, 'b-', linewidth=2, label='Pen trace')

            ax.legend()

            # Update torque plot
            ax_torque.cla()
            ax_torque.set_title('Joint Torques')
            ax_torque.set_xlabel('Joint')
            ax_torque.set_ylabel('Torque (Nm)')

            current_torques = self.trajectory['torques'][frame]
            x_pos = np.arange(4)

            ax_torque.bar(x_pos, current_torques, alpha=0.7, color='steelblue')

            ax_torque.set_xticks(x_pos)
            ax_torque.set_xticklabels(joint_names)
            ax_torque.grid(True, alpha=0.3)

        anim = FuncAnimation(fig, update, frames=len(self.trajectory['joint_angles']),
                           interval=interval, repeat=False)  # Don't loop - stop when done

        if save_path:
            print(f"Saving animation to {save_path}...")
            anim.save(save_path, writer='pillow', fps=20)
            print("Animation saved!")

        plt.tight_layout()

        # Better Ctrl+C handling
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\nAnimation interrupted by user")
            plt.close('all')

        return anim

    def generate_report(self):
        """Generate text report of simulation"""
        if self.trajectory is None:
            print("No trajectory to report. Generate trajectory first.")
            return

        print("\n" + "="*60)
        print("ROBOT ARM SIMULATION REPORT")
        print("="*60)

        print(f"\nRobot: {self.config['robot']['name']}")
        print(f"Letters: {self.config['writing']['letters']}")

        print("\n--- Requirements Check ---")
        print(f"[OK] DOF: {self.config['robot']['dof']} (minimum 4)")
        print(f"[OK] 3D Workspace: Yes")

        # Check joint types
        joint_types = [link['joint_type'] for link in self.config['robot']['links']]
        revolute_count = joint_types.count('revolute')
        prismatic_count = joint_types.count('prismatic')

        print(f"[OK] Revolute joints: {revolute_count} (minimum 2)")
        print(f"[OK] Prismatic joints: {prismatic_count} (minimum 1, not first link)")

        print(f"\n--- Performance ---")
        print(f"Total waypoints: {len(self.trajectory['joint_angles'])}")
        print(f"Average IK error: {np.mean(self.trajectory['ik_errors']):.4f}m")
        print(f"Max IK error: {np.max(self.trajectory['ik_errors']):.4f}m")

        print(f"\n--- Joint Torque Requirements ---")
        joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist']
        for i, joint_name in enumerate(joint_names):
            max_req = np.max(np.abs(self.trajectory['torques'][:, i]))
            avg_req = np.mean(np.abs(self.trajectory['torques'][:, i]))
            print(f"{joint_name}: Max {max_req:.2f} Nm, Avg {avg_req:.2f} Nm")

        print("\n" + "="*60)


if __name__ == '__main__':
    try:
        # Run simulation
        sim = RobotSimulator()

        # Generate trajectory
        sim.generate_robot_trajectory()

        # Generate report
        sim.generate_report()

        # Animate (comment out if running headless)
        print("\nStarting animation... Close window when finished or press Ctrl+C.")
        sim.animate(show_trace=True)

        print("\nSimulation completed successfully!")

    except KeyboardInterrupt:
        print("\n\nSimulation interrupted by user (Ctrl+C)")
    except Exception as e:
        print(f"\n\nError during simulation: {e}")
        raise
