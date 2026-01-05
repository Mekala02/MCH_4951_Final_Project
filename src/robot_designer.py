"""
Robot Designer - Kinematics and Dynamics Module
Handles robot chain creation, FK/IK, and torque calculations
"""

import numpy as np
import yaml
import os
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink


class RobotArm:
    """4-DOF Robot Arm with kinematics and dynamics"""

    def __init__(self, config_path=None):
        """Initialize robot from config file"""
        if config_path is None:
            # Default to config.yaml in the same directory as this script
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.chain = self._build_chain()
        self.g = self.config['simulation']['gravity']

    def _build_chain(self):
        """Build ikpy chain - using simplified working robot instead of DH parameters"""
        # Simple 4-DOF robot that works well with ikpy IK solver
        # Configuration: Base_Rot_Z -> Shoulder_Rot_Y -> Elbow_Rot_Y -> Prismatic_X

        links = [
            OriginLink(),

            # Joint 1: Base rotation around Z (spin)
            URDFLink(
                name="base",
                origin_translation=np.array([0, 0, 0.2]),  # 20cm tall base
                origin_orientation=np.array([0, 0, 0]),
                rotation=np.array([0, 0, 1]),  # Rotate around Z
                bounds=[-3.14, 3.14]
            ),

            # Joint 2: Shoulder rotation around Y (up/down)
            URDFLink(
                name="shoulder",
                origin_translation=np.array([0, 0, 0]),
                origin_orientation=np.array([0, 0, 0]),
                rotation=np.array([0, 1, 0]),  # Rotate around Y
                bounds=[-1.57, 1.57]
            ),

            # Joint 3: Elbow rotation around Y (bend arm)
            URDFLink(
                name="elbow",
                origin_translation=np.array([0.4, 0, 0]),  # 40cm upper arm
                origin_orientation=np.array([0, 0, 0]),
                rotation=np.array([0, 1, 0]),  # Rotate around Y
                bounds=[-1.57, 1.57]
            ),

            # Joint 4: Prismatic (telescoping forearm)
            URDFLink(
                name="wrist",
                origin_translation=np.array([0.2, 0, 0]),  # 20cm base forearm
                origin_orientation=np.array([0, 0, 0]),
                translation=np.array([1, 0, 0]),  # Extend in X direction
                joint_type='prismatic',
                bounds=[0, 0.3]  # Can extend 0-30cm
            ),
        ]

        return Chain(name=self.config['robot']['name'], links=links)

    def forward_kinematics(self, joint_angles):
        """
        Compute end effector position from joint angles

        Args:
            joint_angles: list of 4 joint values [base, shoulder, elbow, wrist]

        Returns:
            4x4 transformation matrix
        """
        # ikpy expects joint angles with origin link, so prepend 0
        full_joints = [0] + list(joint_angles)
        return self.chain.forward_kinematics(full_joints)

    def inverse_kinematics(self, target_position, initial_guess=None):
        """
        Compute joint angles to reach target position

        Args:
            target_position: [x, y, z] target coordinates
            initial_guess: optional initial joint configuration

        Returns:
            Array of joint angles (without origin link)
        """
        if initial_guess is None:
            initial_guess = [0, 0, 0, 0.3, 0]  # Include origin link
        else:
            initial_guess = [0] + list(initial_guess)

        # Solve IK
        ik_solution = self.chain.inverse_kinematics(
            target_position=target_position,
            initial_position=initial_guess
        )

        # Return without origin link
        return ik_solution[1:]

    def get_link_masses(self):
        """Get mass of each link including motors"""
        masses = []
        for i, link_name in enumerate(['base', 'shoulder', 'elbow', 'wrist']):
            link_mass = self.config['link_specs'][link_name]['mass']
            # Note: Motor mass should be added after motor selection
            masses.append(link_mass)

        # Add end effector mass
        masses.append(self.config['end_effector']['mass'])

        return np.array(masses)

    def compute_static_torques(self, joint_angles):
        """
        Compute static torques due to gravity

        Args:
            joint_angles: current joint configuration

        Returns:
            Array of required torques at each joint [Nm]
        """
        masses = self.get_link_masses()
        torques = np.zeros(4)

        # Get positions of each link's center of mass
        # For simplicity, assume CoM at midpoint of each link
        for i in range(4):
            # Get transformation to this joint
            # Need to pad with zeros for remaining joints
            partial_joints = [0] + list(joint_angles[:i+1]) + [0] * (4 - i - 1)
            T = self.chain.forward_kinematics(partial_joints)

            # Position of joint
            pos = T[:3, 3]

            # Distance from base (moment arm)
            r = np.linalg.norm(pos[:2])  # horizontal distance

            # Torque due to gravity (simplified)
            # τ = m * g * r
            torques[i] = masses[i] * self.g * r

        return torques

    def compute_dynamic_torques(self, joint_angles, joint_velocities, joint_accelerations):
        """
        Compute dynamic torques (simplified Euler-Lagrange)

        Args:
            joint_angles: current configuration
            joint_velocities: joint velocities
            joint_accelerations: joint accelerations

        Returns:
            Array of required torques [Nm]
        """
        # Simplified: torque = I * alpha + static torques
        # Using approximation: I ≈ m * r²

        static_torques = self.compute_static_torques(joint_angles)
        masses = self.get_link_masses()

        dynamic_torques = static_torques.copy()

        for i in range(4):
            # Approximate inertia
            partial_joints = [0] + list(joint_angles[:i+1]) + [0] * (4 - i - 1)
            T = self.chain.forward_kinematics(partial_joints)
            pos = T[:3, 3]
            r = np.linalg.norm(pos[:2])

            I_approx = masses[i] * r**2
            dynamic_torques[i] += I_approx * joint_accelerations[i]

        return dynamic_torques

    def check_motor_limits(self, torques):
        """
        Get maximum torques from simulation history

        Args:
            torque_history: list of torque arrays from simulation

        Returns:
            Array of maximum absolute torques for each joint
        """
        torque_array = np.array(torque_history)
        max_torques = np.max(np.abs(torque_array), axis=0)
        return max_torques

    def get_workspace_bounds(self):
        """Calculate approximate workspace bounds"""
        # Maximum reach: sum of link lengths
        links = self.config['robot']['links']
        max_reach = sum([link.get('a', 0) + link.get('d', 0) for link in links])

        return {
            'max_reach': max_reach,
            'min_z': 0,
            'max_z': max_reach
        }

    def visualize(self, joint_angles, ax=None):
        """
        Visualize robot configuration using ikpy's plot

        Args:
            joint_angles: joint configuration to visualize
            ax: matplotlib 3D axis (optional)
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        if ax is None:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')

        # Prepend origin link angle
        full_joints = [0] + list(joint_angles)

        # Use ikpy's built-in plot
        self.chain.plot(full_joints, ax)

        # Add cardboard position
        cardboard_pos = self.config['workspace']['cardboard_position']
        cardboard_w = self.config['workspace']['cardboard_width']
        cardboard_h = self.config['workspace']['cardboard_height']

        # Draw cardboard as a vertical rectangle in XZ plane (Y constant)
        # Corners: bottom-left, bottom-right, top-right, top-left, back to bottom-left
        x = [cardboard_pos[0] - cardboard_w/2, cardboard_pos[0] + cardboard_w/2,
             cardboard_pos[0] + cardboard_w/2, cardboard_pos[0] - cardboard_w/2,
             cardboard_pos[0] - cardboard_w/2]
        y = [cardboard_pos[1]] * 5  # Y constant (flat vertical surface)
        z = [cardboard_pos[2] - cardboard_h/2, cardboard_pos[2] - cardboard_h/2,
             cardboard_pos[2] + cardboard_h/2, cardboard_pos[2] + cardboard_h/2,
             cardboard_pos[2] - cardboard_h/2]

        ax.plot(x, y, z, 'r-', linewidth=2, label='Cardboard (vertical)', alpha=0.7)

        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.legend()

        return ax


if __name__ == '__main__':
    # Test robot creation and visualization
    robot = RobotArm()

    print(f"Robot: {robot.config['robot']['name']}")
    print(f"DOF: {robot.config['robot']['dof']}")
    print(f"Workspace max reach: {robot.get_workspace_bounds()['max_reach']:.2f}m")

    # Test FK with home position
    home_position = [0, 0, 0.3, 0]  # [base, shoulder, elbow, wrist]
    T = robot.forward_kinematics(home_position)
    end_pos = T[:3, 3]
    print(f"\nHome position end effector: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")

    # Visualize
    import matplotlib.pyplot as plt
    robot.visualize(home_position)
    plt.title('Robot Arm - Home Position')
    plt.show()
