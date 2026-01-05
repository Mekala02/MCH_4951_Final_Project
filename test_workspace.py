"""Test robot workspace and find good cardboard position"""

from robot_designer import RobotArm
import numpy as np

robot = RobotArm()

print("Testing robot forward kinematics at various configurations...")
print("\nRobot link lengths:")
for link in robot.config['robot']['links']:
    print(f"  {link['name']}: a={link['a']}, d={link['d']}")

# Test some configurations
configs = [
    ([0, 0, 0.3, 0], "Home"),
    ([0, 0.5, 0.3, 0], "Shoulder forward"),
    ([0, -0.5, 0.3, 0], "Shoulder back"),
    ([1.57, 0, 0.3, 0], "Base rotated 90°"),
    ([0, 0, 0.1, 0], "Elbow retracted"),
    ([0, 0, 0.5, 0], "Elbow extended"),
]

print("\nReachable positions:")
for joints, desc in configs:
    T = robot.forward_kinematics(joints)
    pos = T[:3, 3]
    print(f"  {desc:20s}: X={pos[0]:6.3f}, Y={pos[1]:6.3f}, Z={pos[2]:6.3f}")

print("\nSuggested cardboard position:")
print("  Position: [0.5, 0.4, 0.4] (50cm forward, 40cm to right, 40cm up)")
print("  This should be reachable by the robot")
