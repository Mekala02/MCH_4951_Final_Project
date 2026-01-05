"""Simplified robot with better IK - just test if this works better"""

import numpy as np
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot as plt

# Build a simple 4-DOF robot that ikpy can handle
# Simple serial manipulator: Base_Rot -> Shoulder_Rot -> Elbow_Prismatic -> Wrist_Rot

my_chain = Chain(name='simple_4dof', links=[
    OriginLink(),

    # Joint 1: Base rotation around Z
    URDFLink(
        name="base_rotation",
        origin_translation=[0, 0, 0.2],  # 20cm tall base
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1]  # Rotate around Z
    ),

    # Joint 2: Shoulder rotation around Y (to lift arm up/down)
    URDFLink(
        name="shoulder",
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],  # Rotate around Y
        bounds=[-1.57, 1.57]
    ),

    # Link extending forward
    URDFLink(
        name="upper_arm",
        origin_translation=[0.4, 0, 0],  # 40cm forward
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],  # Another Y rotation
        bounds=[-1.57, 1.57]
    ),

    # Joint 3: Prismatic (telescoping)
    URDFLink(
        name="forearm_prismatic",
        origin_translation=[0.2, 0, 0],  # Base length 20cm
        origin_orientation=[0, 0, 0],
        translation=[1, 0, 0],  # Extend in X direction
        joint_type='prismatic',
        bounds=[0, 0.3]  # Can extend 0-30cm
    ),
])

# Test it
print("Testing simple robot...")
print(f"Chain has {len(my_chain.links)} links")

# Home position
home = [0, 0, 0, 0, 0]
T = my_chain.forward_kinematics(home)
pos = T[:3, 3]
print(f"Home position: {pos}")

# Test IK
target = [0.5, 0, 0.4]
print(f"\nTarget: {target}")
ik_result = my_chain.inverse_kinematics(target)
print(f"IK solution: {ik_result}")

# Verify
T_verify = my_chain.forward_kinematics(ik_result)
reached = T_verify[:3, 3]
print(f"Reached: {reached}")
print(f"Error: {np.linalg.norm(reached - target):.6f}m")

# Visualize
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
my_chain.plot(ik_result, ax, target=target)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.title('Simple 4-DOF Robot')
print("\nVisualization ready - close window to exit")
plt.show()
