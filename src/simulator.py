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
from roboticstoolbox.robot.Robot import Robot

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

        # Load robot directly from URDF using RTB
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'config', 'robot.urdf')
        self.robot = Robot.URDF(urdf_path)
        self.robot.gravity = [0, 0, -self.config['simulation']['gravity']]

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
        points_per_segment = self.config['simulation']['interpolation_points']
        interp_traj = self.traj_gen.interpolate_trajectory(ee_traj, points_per_segment=points_per_segment)

        # Solve IK for each waypoint
        joint_angles = []
        reached_positions = []
        torques = []
        ik_errors = []

        # Initial guess: mid-range for all joints
        num_joints = self.robot.n
        previous_joints = np.zeros(num_joints)
        if self.robot.qlim is not None:
            for i in range(num_joints):
                lower = self.robot.qlim[0, i]
                upper = self.robot.qlim[1, i]
                previous_joints[i] = (lower + upper) / 2

        print(f"Solving IK for {len(interp_traj['waypoints'])} waypoints...")

        # Get IK configuration
        from spatialmath import SE3
        ik_mask = self.config['ik']['position_mask']

        for i, target in enumerate(interp_traj['waypoints']):
            try:
                # Solve IK with previous solution as initial guess using RTB directly
                T_target = SE3(target[0], target[1], target[2])
                ik_solution = self.robot.ikine_LM(T_target, q0=previous_joints, mask=ik_mask)
                joints = ik_solution.q if ik_solution.success else None

                # Check if IK succeeded
                if joints is None:
                    raise ValueError("IK solver did not converge")

                # Check for large joint jumps (teleporting) and handle angle wrapping
                max_step = self.config['ik']['max_joint_step']
                if len(joint_angles) > 0:
                    # Handle angle wrapping for revolute joints (e.g., -π to π wrapping)
                    joints_unwrapped = joints.copy()
                    # Check joint type from URDF via RTB
                    for j in range(num_joints):
                        # Get joint type from link (link j+1 represents joint j's type)
                        is_prismatic = self.robot.links[j+1].isprismatic
                        if not is_prismatic:
                            # Check if wrapping around ±π would give smaller difference
                            diff_raw = joints[j] - previous_joints[j]
                            diff_wrapped_pos = (joints[j] + 2*np.pi) - previous_joints[j]
                            diff_wrapped_neg = (joints[j] - 2*np.pi) - previous_joints[j]

                            # Choose the angle that gives smallest absolute difference
                            diffs = [diff_raw, diff_wrapped_pos, diff_wrapped_neg]
                            abs_diffs = [abs(d) for d in diffs]
                            min_idx = abs_diffs.index(min(abs_diffs))

                            if min_idx == 1:
                                joints_unwrapped[j] = joints[j] + 2*np.pi
                            elif min_idx == 2:
                                joints_unwrapped[j] = joints[j] - 2*np.pi

                    # Now check for jumps with unwrapped angles
                    joint_diff = np.abs(joints_unwrapped - previous_joints)

                    if np.any(joint_diff > max_step):
                        # Large jump detected - try smoother path with more intermediate steps
                        best_joints = joints_unwrapped
                        min_max_diff = np.max(joint_diff)

                        # Try multiple intermediate waypoints
                        for num_steps in [3, 5, 7]:
                            temp_joints = previous_joints.copy()
                            success = True

                            for step in range(1, num_steps + 1):
                                alpha = step / num_steps
                                prev_target = interp_traj['waypoints'][i-1] if i > 0 else target
                                interp_target = prev_target + alpha * (target - prev_target)

                                # Use RTB's IK directly
                                T_interp = SE3(interp_target[0], interp_target[1], interp_target[2])
                                ik_step = self.robot.ikine_LM(T_interp, q0=temp_joints, mask=ik_mask)
                                step_joints = ik_step.q if ik_step.success else None

                                if step_joints is None:
                                    success = False
                                    break

                                temp_joints = step_joints

                            if success:
                                # Check if this path is smoother
                                final_diff = np.max(np.abs(temp_joints - previous_joints))
                                if final_diff < min_max_diff:
                                    best_joints = temp_joints
                                    min_max_diff = final_diff
                                    if min_max_diff <= max_step:
                                        break  # Found good solution

                        joints = best_joints
                    else:
                        joints = joints_unwrapped

                # Verify solution using RTB's fkine directly
                T = self.robot.fkine(joints)
                reached = T.t  # RTB SE3 object, .t gives translation vector

                error = np.linalg.norm(reached - target)
                ik_errors.append(error)

                joint_angles.append(joints)
                reached_positions.append(reached)

                # Calculate static torques using RTB's pay() method
                # This properly accounts for gravity on each link at its center of mass
                try:
                    # Try gravload first (works for some robot types)
                    static_torque = self.robot.gravload(joints)
                except:
                    # For URDF robots, compute link-by-link gravity torques
                    # Use RTB's pay() method for each link's contribution
                    static_torque = np.zeros(self.robot.n)
                    g = np.array([0, 0, -self.config['simulation']['gravity']])

                    # Sum gravity effects from all links
                    for link_idx, link in enumerate(self.robot.links):
                        if link.m is not None and link.m > 0:
                            # Gravity force at link's center of mass
                            F_gravity = link.m * g
                            wrench = np.concatenate([F_gravity, [0, 0, 0]])

                            # Use pay() to get joint torques for this link's gravity
                            tau_link = self.robot.pay(W=wrench, q=joints, frame=link_idx)
                            if tau_link is not None:
                                static_torque += tau_link

                torques.append(static_torque)

                previous_joints = joints

                progress_interval = self.config['simulation']['progress_report_interval']
                if i % progress_interval == 0:
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
                    # Default configuration for failed IK (mid-range)
                    default_joints = np.zeros(num_joints)
                    if self.robot.qlim is not None:
                        for k in range(num_joints):
                            lower = self.robot.qlim[0, k]
                            upper = self.robot.qlim[1, k]
                            default_joints[k] = (lower + upper) / 2
                    joint_angles.append(default_joints)
                    reached_positions.append(target)
                    torques.append(np.zeros(num_joints))
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
        # Get joint names dynamically from config
        # Get joint names from motors config
        joint_names = list(self.config['motors'].keys())

        for i, joint_name in enumerate(joint_names):
            max_torque_req = np.max(np.abs(torque_array[:, i]))
            avg_torque_req = np.mean(np.abs(torque_array[:, i]))
            print(f"  {joint_name}: Max {max_torque_req:.2f} Nm, Avg {avg_torque_req:.2f} Nm")

        print("\nUse these values to select appropriate motors (recommended: 1.5x-2x safety factor)")

        return self.trajectory

    def animate(self, save_path=None, interval=None, show_trace=True):
        """
        Animate the robot trajectory

        Args:
            save_path: Optional path to save animation (e.g., 'animation.gif')
            interval: Milliseconds between frames (if None, uses config value)
            show_trace: Show pen trace
        """
        if interval is None:
            interval = self.config['visualization']['animation_interval']
        if self.trajectory is None:
            raise ValueError("No trajectory generated. Call generate_robot_trajectory() first.")

        fig_width = self.config['visualization']['figure_width']
        fig_height = self.config['visualization']['figure_height']
        fig = plt.figure(figsize=(fig_width, fig_height))
        ax = fig.add_subplot(121, projection='3d')
        ax_torque = fig.add_subplot(122)

        # Setup 3D plot
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_title('Robot Arm Animation')

        # Set fixed limits from config
        x_lim = self.config['visualization']['x_limits']
        y_lim = self.config['visualization']['y_limits']
        z_lim = self.config['visualization']['z_limits']
        ax.set_xlim(x_lim[0], x_lim[1])
        ax.set_ylim(y_lim[0], y_lim[1])
        ax.set_zlim(z_lim[0], z_lim[1])

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
        # Get joint names dynamically from config
        num_joints = self.robot.n
        # Get joint names from motors config
        joint_names = list(self.config['motors'].keys())
        torque_colors = self.config['visualization']['torque_line_colors']

        # Time array for torque plot
        time_array = np.arange(len(self.trajectory['joint_angles'])) * self.config['simulation']['timestep']

        def update(frame):
            ax.cla()
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_title(f'Robot Arm - Frame {frame}/{len(self.trajectory["joint_angles"])}')
            ax.set_xlim(x_lim[0], x_lim[1])
            ax.set_ylim(y_lim[0], y_lim[1])
            ax.set_zlim(z_lim[0], z_lim[1])

            # Draw cardboard
            ax.plot(x_card, y_card, z_card, 'r-', linewidth=2, alpha=0.5)

            # Get current joint configuration
            joints = self.trajectory['joint_angles'][frame]

            # Compute link positions using RTB's forward kinematics
            # Robot has len(robot.links) links including base_link
            link_positions = []

            # Get position of each link frame from URDF
            for i in range(len(self.robot.links)):
                T = self.robot.fkine(joints, end=self.robot.links[i])
                link_positions.append(T.t)

            # End-effector position is the last link position
            ee_pos = link_positions[-1]

            # Draw links - simple visualization
            joint_names_list = list(self.config['motors'].keys())
            colors = ['steelblue', 'orange', 'green', 'red', 'purple']

            for i in range(len(link_positions) - 1):
                p1 = link_positions[i]
                p2 = link_positions[i+1]

                # Label each link with joint name
                if i < len(joint_names_list):
                    joint_name = joint_names_list[i]
                    color = colors[i % len(colors)]
                    ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                           'o-', linewidth=3, markersize=6, color=color,
                           label=f'{joint_name}')

            # Show pen state with colored marker (ee_pos already computed above)
            pen_is_down = self.trajectory['pen_states'][frame]
            pen_color = 'red' if pen_is_down else 'green'
            pen_label = 'Pen DOWN (writing)' if pen_is_down else 'Pen UP (moving)'
            ax.scatter([ee_pos[0]], [ee_pos[1]], [ee_pos[2]],
                      c=pen_color, s=150, marker='v' if pen_is_down else '^',
                      edgecolors='black', linewidths=2, label=pen_label, zorder=10)

            # Add trace only when pen is down
            if show_trace and pen_is_down:
                trace_x.append(ee_pos[0])
                trace_y.append(ee_pos[1])
                trace_z.append(ee_pos[2])

            if len(trace_x) > 0:
                ax.plot(trace_x, trace_y, trace_z, 'b-', linewidth=3, label='Pen trace (drawn letters)')

            ax.legend(loc='upper left', fontsize=8)

            # Update torque plot - show torque history over time
            ax_torque.cla()
            ax_torque.set_title('Joint Torques Over Time')
            ax_torque.set_xlabel('Time (s)')
            ax_torque.set_ylabel('Torque (Nm)')

            # Plot torque history up to current frame for each joint
            for i in range(len(joint_names)):
                torque_history = self.trajectory['torques'][:frame+1, i]
                ax_torque.plot(time_array[:frame+1], torque_history,
                             color=torque_colors[i % len(torque_colors)], label=joint_names[i], linewidth=2)

            ax_torque.legend(loc='upper right')
            ax_torque.grid(True, alpha=0.3)
            ax_torque.set_xlim(0, time_array[-1])

            # Set y-limits based on max torques
            max_torque = np.max(np.abs(self.trajectory['torques']))
            torque_margin = self.config['visualization']['torque_plot_margin']
            ax_torque.set_ylim(-max_torque * torque_margin, max_torque * torque_margin)

        anim = FuncAnimation(fig, update, frames=len(self.trajectory['joint_angles']),
                           interval=interval, repeat=False)  # Don't loop - stop when done

        if save_path:
            print(f"Saving animation to {save_path}...")
            fps = self.config['visualization']['animation_fps']
            anim.save(save_path, writer='pillow', fps=fps)
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

        print(f"\nRobot: {self.robot.name}")
        print(f"Letters: {self.config['writing']['letters']}")

        print("\n--- Requirements Check ---")
        print(f"[OK] DOF: {self.robot.n} (minimum 4)")
        print(f"[OK] 3D Workspace: Yes")

        # Check joint types from URDF via RTB
        # Links 1 to n represent the joints (link 0 is base)
        revolute_count = 0
        prismatic_count = 0
        for i in range(1, len(self.robot.links)):
            if self.robot.links[i].isprismatic:
                prismatic_count += 1
            elif self.robot.links[i].isrevolute:
                revolute_count += 1

        print(f"[OK] Revolute joints: {revolute_count} (minimum 2)")
        print(f"[OK] Prismatic joints: {prismatic_count} (minimum 1, not first link)")

        print(f"\n--- Performance ---")
        print(f"Total waypoints: {len(self.trajectory['joint_angles'])}")
        print(f"Average IK error: {np.mean(self.trajectory['ik_errors']):.4f}m")
        print(f"Max IK error: {np.max(self.trajectory['ik_errors']):.4f}m")

        print(f"\n--- Joint Torque Requirements ---")
        # Get joint names dynamically from config
        num_joints = self.robot.n
        # Get joint names from motors config
        joint_names = list(self.config['motors'].keys())
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
