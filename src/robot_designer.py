"""
Robot Designer - Kinematics and Dynamics Module
Handles robot chain creation, FK/IK, and torque calculations
"""

import numpy as np
import yaml
import os
import roboticstoolbox as rtb
from spatialmath import SE3


class RobotArm:
    """4-DOF Robot Arm with kinematics and dynamics"""

    def __init__(self, config_path=None):
        """Initialize robot from config file"""
        if config_path is None:
            # Default to config.yaml in the same directory as this script
            config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')

        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        self.g = self.config['simulation']['gravity']
        self.robot = self._build_rtb_robot()  # Single RTB robot

    def _build_rtb_robot(self):
        """
        Build RTB robot for accurate dynamics calculations
        """
        # Get total lumped masses (link + motor)
        masses = self._get_total_masses()

        # Build RTB links using Elementary Transform Sequence (ETS)
        import roboticstoolbox as rtb
        from roboticstoolbox import ET

        # Compact 4-DOF: Base_Rz -> Shoulder_Ry -> Elbow_Prismatic_X -> Wrist_Ry
        # Design: Compact, well-proportioned links for stability and balance

        # Get joint limits from config
        base_limits = self.config['robot']['links'][0]['limits']
        shoulder_limits = self.config['robot']['links'][1]['limits']
        elbow_limits = self.config['robot']['links'][2]['limits']
        wrist_limits = self.config['robot']['links'][3]['limits']

        # Link 1: Base rotation (revolute Z) - compact stable base
        L1 = rtb.Link(
            ET.tz(0.10) * ET.Rz(),  # Lift 0.10m (compact, stable), then rotate around Z
            name="base",
            m=masses[0],
            r=[0, 0, 0.05],  # COM at middle of 0.10m base
            I=self._compute_cylinder_inertia(masses[0], 0.03, 0.10),
            qlim=base_limits  # ±160° realistic limit
        )

        # Link 2: Shoulder (revolute Y) - compact shoulder link
        # Extends 0.25m along local X before the joint
        L2 = rtb.Link(
            ET.tx(0.25) * ET.Ry(),
            name="shoulder",
            m=masses[1],
            r=[0.125, 0, 0],  # COM at middle of 0.25m link
            I=self._compute_cylinder_inertia(masses[1], 0.025, 0.25),
            qlim=shoulder_limits  # -80° to +80° prevents collisions
        )

        # Link 3: Elbow (prismatic X) - compact prismatic extension
        L3 = rtb.Link(
            ET.tx(),  # Prismatic extension
            name="elbow",
            m=masses[2],
            r=[0.15, 0, 0],  # COM at middle of extension range (0.20 to 0.40, mid=0.30)
            I=self._compute_cylinder_inertia(masses[2], 0.02, 0.20),
            qlim=elbow_limits,  # 20-40cm mechanical stops
            isprismatic=True
        )

        # Link 4: Wrist (revolute Y) - compact wrist link
        # Extends 0.15m before the joint
        L4 = rtb.Link(
            ET.tx(0.15) * ET.Ry(),
            name="wrist",
            m=masses[3],
            r=[0.075, 0, 0],  # COM at middle of 0.15m link
            I=self._compute_cylinder_inertia(masses[3], 0.015, 0.15),
            qlim=wrist_limits  # ±120° realistic limit
        )

        # Create robot
        robot = rtb.Robot([L1, L2, L3, L4], name=self.config['robot']['name'])
        robot.gravity = [0, 0, -self.g]  # Set gravity vector

        return robot

    def _get_total_masses(self):
        """
        Compute total lumped mass for each link (link + motor + end effector)
        Masses concentrated at link COM
        """
        masses = []
        joint_names = ['base', 'shoulder', 'elbow', 'wrist']

        for joint_name in joint_names:
            link_mass = self.config['link_specs'][joint_name]['mass']
            motor_mass = self.config['motors'][joint_name]['mass']
            total_mass = link_mass + motor_mass

            # Add end effector mass to last link
            if joint_name == 'wrist':
                total_mass += self.config['end_effector']['mass']

            masses.append(total_mass)

        return np.array(masses)

    def _compute_cylinder_inertia(self, mass, radius, height):
        """
        Compute inertia tensor for solid cylinder about its COM

        Args:
            mass: total mass (kg)
            radius: cylinder radius (m)
            height: cylinder height (m)

        Returns:
            3x3 inertia matrix
        """
        # For solid cylinder:
        # Ixx = Iyy = m*(3r² + h²)/12  (perpendicular to axis)
        # Izz = m*r²/2                 (about cylinder axis)

        Ixx = Iyy = mass * (3 * radius**2 + height**2) / 12
        Izz = mass * radius**2 / 2

        return np.diag([Ixx, Iyy, Izz])

    def forward_kinematics(self, joint_angles):
        """
        Compute end effector pose from joint angles using RTB

        Args:
            joint_angles: array of 4 joint values [base, shoulder, elbow, wrist]

        Returns:
            SE3 transformation object (can get 4x4 matrix with .A attribute)
        """
        T = self.robot.fkine(joint_angles)
        return T

    def inverse_kinematics(self, target_position, initial_guess=None):
        """
        Compute joint angles to reach target position using RTB's IK

        Args:
            target_position: [x, y, z] target coordinates
            initial_guess: optional initial joint configuration [4,]

        Returns:
            Array of joint angles [4,] or None if IK fails
        """
        if initial_guess is None:
            # Smart initial guess based on target position
            # Base angle points toward target in XY plane
            base_angle = np.arctan2(target_position[1], target_position[0])
            initial_guess = np.array([base_angle, 0, 0.30, 0])  # Mid-range prismatic (0.20-0.40)

        # Create SE3 transform from target position
        # RTB IK needs full 6-DOF pose, but we only care about position
        # Keep orientation fixed (end effector pointing forward)
        T_target = SE3(target_position[0], target_position[1], target_position[2])

        # Use Levenberg-Marquardt IK solver (robust, handles singularities well)
        ik_solution = self.robot.ikine_LM(
            T_target,
            q0=initial_guess,
            mask=[1, 1, 1, 0, 0, 0]  # Only constrain position (x,y,z), not orientation
        )

        # Check if solution was found
        if ik_solution.success:
            return ik_solution.q
        else:
            # If LM fails, try numerical IK as backup
            ik_solution = self.robot.ikine_NR(
                T_target,
                q0=initial_guess,
                mask=[1, 1, 1, 0, 0, 0]
            )
            if ik_solution.success:
                return ik_solution.q
            else:
                return None  # IK failed

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
        Compute static torques due to gravity using RTB's Recursive Newton-Euler

        Args:
            joint_angles: current joint configuration [4,]

        Returns:
            Array of required torques at each joint [Nm]
        """
        # Static case: zero velocities and accelerations
        qd = np.zeros(4)
        qdd = np.zeros(4)

        # Use RTB's rne (Recursive Newton-Euler) for inverse dynamics
        # Returns torques needed to maintain position against gravity
        tau = self.robot.rne(joint_angles, qd, qdd)

        return tau

    def compute_dynamic_torques(self, joint_angles, joint_velocities, joint_accelerations):
        """
        Compute dynamic torques using RTB's Recursive Newton-Euler
        Includes: gravity + inertial + Coriolis + centrifugal forces

        Args:
            joint_angles: current configuration [4,]
            joint_velocities: joint velocities [4,]
            joint_accelerations: joint accelerations [4,]

        Returns:
            Array of required torques [Nm]
        """
        # Use RTB's rne for full inverse dynamics
        # tau = M(q)*qdd + C(q,qd)*qd + G(q)
        tau = self.robot.rne(joint_angles, joint_velocities, joint_accelerations)

        return tau

    def check_motor_limits(self, torques):
        """
        Check if torques exceed motor limits

        Args:
            torques: Array of torques to check [Nm]

        Returns:
            tuple: (all_ok: bool, exceeded_list: list of dicts)
        """
        joint_names = ['base', 'shoulder', 'elbow', 'wrist']
        exceeded = []

        for i, (joint_name, torque) in enumerate(zip(joint_names, torques)):
            max_torque = self.config['motors'][joint_name]['stall_torque']
            if abs(torque) > max_torque:
                exceeded.append({
                    'motor': joint_name,
                    'required': abs(torque),
                    'max': max_torque
                })

        return (len(exceeded) == 0, exceeded)

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
        Visualize robot configuration using RTB's plot

        Args:
            joint_angles: joint configuration to visualize [4,]
            ax: matplotlib 3D axis (optional)
        """
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D

        if ax is None:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')

        # Use RTB's plot method
        self.robot.plot(joint_angles, backend='pyplot', block=False, ax=ax)

        # Add cardboard position
        cardboard_pos = self.config['workspace']['cardboard_position']
        cardboard_w = self.config['workspace']['cardboard_width']
        cardboard_h = self.config['workspace']['cardboard_height']

        # Draw cardboard as a vertical rectangle in XZ plane (Y constant)
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
    home_position = np.array([0, 0, 0.30, 0])  # [base, shoulder, elbow(prismatic), wrist]
    T = robot.forward_kinematics(home_position)
    end_pos = T.t  # RTB SE3 object has .t for translation
    print(f"\nHome position end effector: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")

    # Visualize
    import matplotlib.pyplot as plt
    robot.visualize(home_position)
    plt.title('Robot Arm - Home Position (RTB)')
    plt.show()
