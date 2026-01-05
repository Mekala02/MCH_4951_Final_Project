"""Quick test of robot_designer without GUI"""

from robot_designer import RobotArm
import numpy as np

# Test robot creation
robot = RobotArm()

print(f"Robot: {robot.config['robot']['name']}")
print(f"DOF: {robot.config['robot']['dof']}")
print(f"Workspace max reach: {robot.get_workspace_bounds()['max_reach']:.2f}m")

# Test FK with home position
home_position = [0, 0, 0.3, 0]  # [base, shoulder, elbow, wrist]
T = robot.forward_kinematics(home_position)
end_pos = T[:3, 3]
print(f"\nHome position end effector: [{end_pos[0]:.3f}, {end_pos[1]:.3f}, {end_pos[2]:.3f}]")

# Test IK
target = [0.5, 0.0, 0.5]  # Try to reach a point in front
print(f"\nTesting IK for target: {target}")
try:
    ik_result = robot.inverse_kinematics(target)
    print(f"IK solution: {ik_result}")

    # Verify with FK
    T_verify = robot.forward_kinematics(ik_result)
    reached_pos = T_verify[:3, 3]
    print(f"Reached position: [{reached_pos[0]:.3f}, {reached_pos[1]:.3f}, {reached_pos[2]:.3f}]")
    error = np.linalg.norm(reached_pos - target)
    print(f"Position error: {error:.4f}m")
except Exception as e:
    print(f"IK failed: {e}")

# Test torque calculation
print(f"\nStatic torques at home position:")
torques = robot.compute_static_torques(home_position)
for i, torque in enumerate(torques):
    print(f"  Joint {i}: {torque:.3f} Nm")

# Check motor limits
ok, exceeded = robot.check_motor_limits(torques)
if ok:
    print("All torques within motor limits!")
else:
    print("Warning: Some motors exceeded:")
    for ex in exceeded:
        print(f"  {ex['motor']}: required {ex['required']:.2f} Nm, max {ex['max']:.2f} Nm")

print("\nRobot designer test complete!")
