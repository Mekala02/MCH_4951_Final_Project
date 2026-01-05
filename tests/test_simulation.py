"""Test simulation without GUI"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from simulator import RobotSimulator

# Run simulation
sim = RobotSimulator()

# Generate trajectory
sim.generate_robot_trajectory()

# Generate report
sim.generate_report()

print("\nSimulation complete! Run 'venv/Scripts/python.exe simulator.py' to see animation.")
