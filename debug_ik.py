"""Debug IK issues"""

from robot_designer import RobotArm
import numpy as np

robot = RobotArm()

# Test IK for a simple point on the cardboard
cardboard_pos = robot.config['workspace']['cardboard_position']
print(f"Cardboard position: {cardboard_pos}")

# Try to reach a point on the cardboard
test_point = [0.5, 0.0, 0.7]  # Simple point
print(f"\nTarget point: {test_point}")

# Solve IK
joints = robot.inverse_kinematics(test_point)
print(f"IK solution: {joints}")

# Verify
T = robot.forward_kinematics(joints)
reached = T[:3, 3]
print(f"Reached: {reached}")
print(f"Error: {np.linalg.norm(reached - test_point):.4f}m")

# The problem: ikpy IK solver may not handle our robot well
# Let's check the robot chain
print(f"\nRobot chain has {len(robot.chain.links)} links")
for i, link in enumerate(robot.chain.links):
    print(f"  Link {i}: {link.name}, type: {link}")
