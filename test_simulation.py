"""Test simulation without GUI"""

from simulator import RobotSimulator

# Run simulation
sim = RobotSimulator()

# Generate trajectory
sim.generate_robot_trajectory()

# Generate report
sim.generate_report()

print("\nSimulation complete! Run 'venv/Scripts/python.exe simulator.py' to see animation.")
