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
        Build robot dynamically from config - works for ANY number of links!
        Automatically constructs robot based on config parameters
        """
        masses = self._get_total_masses()
        import roboticstoolbox as rtb
        from roboticstoolbox import ET

        links_config = self.config['robot']['links']
        robot_links = []

        for i, link_cfg in enumerate(links_config):
            # Build Elementary Transform Sequence (ETS) dynamically
            ets_parts = []

            # Fixed translations
            if link_cfg.get('d', 0) != 0:
                ets_parts.append(ET.tz(link_cfg['d']))
            if link_cfg.get('a', 0) != 0:
                ets_parts.append(ET.tx(link_cfg['a']))

            # Offset (if specified)
            if 'offset' in link_cfg and link_cfg['offset'] != 0:
                ets_parts.append(ET.tz(link_cfg['offset']))

            # Fixed rotation (alpha - twist)
            if link_cfg.get('alpha', 0) != 0:
                ets_parts.append(ET.Rx(link_cfg['alpha']))

            # Variable joint (MUST BE LAST in ETS)
            is_prismatic = (link_cfg['joint_type'] == 'prismatic')
            if is_prismatic:
                ets_parts.append(ET.tx())  # Prismatic along X
            else:
                # Determine rotation axis from joint_type comment or default to Rz
                if 'Ry' in str(link_cfg.get('joint_type', '')):
                    ets_parts.append(ET.Ry())
                elif 'Rx' in str(link_cfg.get('joint_type', '')):
                    ets_parts.append(ET.Rx())
                else:
                    ets_parts.append(ET.Rz())  # Default

            # Combine all ETS parts
            ets = ets_parts[0]
            for part in ets_parts[1:]:
                ets *= part

            # Compute center of mass
            a = link_cfg.get('a', 0)
            d = link_cfg.get('d', 0)
            offset = link_cfg.get('offset', 0)

            if is_prismatic:
                # For prismatic: COM at mid-range of extension
                max_ext = link_cfg['limits'][1]
                r = [a + max_ext/2, 0, offset/2]
                length = a + max_ext
            else:
                r = [a/2, 0, (d + offset)/2]
                length = max(a, d)

            # Compute inertia
            diameter = link_cfg['diameter']
            inertia = self._compute_cylinder_inertia(masses[i], diameter, length if length > 0 else diameter)

            # Create link
            robot_link = rtb.Link(
                ets,
                name=link_cfg['name'],
                m=masses[i],
                r=r,
                I=inertia,
                qlim=link_cfg['limits'],
                isprismatic=is_prismatic
            )
            robot_links.append(robot_link)

        robot = rtb.Robot(robot_links, name=self.config['robot']['name'])
        robot.gravity = [0, 0, -self.g]
        return robot

    def _get_total_masses(self):
        """
        Compute total lumped mass for each link (link + motor + end effector)
        Masses concentrated at link COM
        """
        masses = []
        links_config = self.config['robot']['links']

        for i, link in enumerate(links_config):
            link_mass = link['mass']
            motor_mass = link['motor']['mass']
            total_mass = link_mass + motor_mass

            # Add end effector mass to last link
            if i == len(links_config) - 1:  # Last link
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
            joint_angles: array of joint values (num_joints)

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
            initial_guess: optional initial joint configuration

        Returns:
            Array of joint angles or None if IK fails
        """
        if initial_guess is None:
            # Smart initial guess
            base_angle = np.arctan2(target_position[1], target_position[0])
            num_joints = len(self.robot.links)
            initial_guess = np.zeros(num_joints)
            initial_guess[0] = base_angle
            # Set prismatic joint (wrist_extend, index 4) to mid-range
            for i, link in enumerate(self.config['robot']['links']):
                if link['joint_type'] == 'prismatic':
                    limits = link['limits']
                    initial_guess[i] = (limits[0] + limits[1]) / 2

        # Create SE3 transform from target position
        # RTB IK needs full 6-DOF pose, but we only care about position
        # Keep orientation fixed (end effector pointing forward)
        T_target = SE3(target_position[0], target_position[1], target_position[2])

        # Use Levenberg-Marquardt IK solver (robust, handles singularities well)
        ik_mask = self.config['ik']['position_mask']
        ik_solution = self.robot.ikine_LM(
            T_target,
            q0=initial_guess,
            mask=ik_mask  # Only constrain position (x,y,z), not orientation
        )

        # Check if solution was found
        if ik_solution.success:
            return ik_solution.q
        else:
            # If LM fails, try numerical IK as backup
            ik_solution = self.robot.ikine_NR(
                T_target,
                q0=initial_guess,
                mask=ik_mask
            )
            if ik_solution.success:
                return ik_solution.q
            else:
                return None  # IK failed

    def get_link_masses(self):
        """Get mass of each link including motors"""
        masses = []
        for link_config in self.config['robot']['links']:
            link_name = link_config['name']
            link_mass = self.config['link_specs'][link_name]['mass']
            masses.append(link_mass)

        # Add end effector mass
        masses.append(self.config['end_effector']['mass'])

        return np.array(masses)

    def compute_static_torques(self, joint_angles):
        """
        Compute static torques due to gravity using RTB's Recursive Newton-Euler

        Args:
            joint_angles: current joint configuration

        Returns:
            Array of required torques at each joint [Nm]
        """
        # Static case: zero velocities and accelerations
        num_joints = len(self.robot.links)
        qd = np.zeros(num_joints)
        qdd = np.zeros(num_joints)

        # Use RTB's rne (Recursive Newton-Euler) for inverse dynamics
        # Returns torques needed to maintain position against gravity
        tau = self.robot.rne(joint_angles, qd, qdd)

        return tau

    def compute_dynamic_torques(self, joint_angles, joint_velocities, joint_accelerations):
        """
        Compute dynamic torques using RTB's Recursive Newton-Euler
        Includes: gravity + inertial + Coriolis + centrifugal forces

        Args:
            joint_angles: current configuration
            joint_velocities: joint velocities
            joint_accelerations: joint accelerations

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
        joint_names = [link['name'] for link in self.config['robot']['links']]
        exceeded = []

        for i, (link, torque) in enumerate(zip(self.config['robot']['links'], torques)):
            joint_name = link['name']
            max_torque = link['motor']['stall_torque']
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
            fig_width = self.config['visualization']['figure_width']
            fig_height = self.config['visualization']['figure_height']
            fig = plt.figure(figsize=(fig_width, fig_height))
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
    num_joints = len(robot.robot.links)
    home_position = np.zeros(num_joints)
    # Set prismatic joint to mid-range
    for i, link in enumerate(robot.config['robot']['links']):
        if link['joint_type'] == 'prismatic':
            limits = link['limits']
            home_position[i] = (limits[0] + limits[1]) / 2

    T = robot.forward_kinematics(home_position)
    end_pos = T.t  # RTB SE3 object has .t for translation
    print(f"\nHome position end effector: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")

    # Visualize
    import matplotlib.pyplot as plt
    robot.visualize(home_position)
    plt.title('Robot Arm - Home Position (RTB)')
    plt.show()
