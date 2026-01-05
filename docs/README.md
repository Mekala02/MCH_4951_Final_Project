# Robot Arm Simulator - MCH 4951 Final Project

A Python-based 4-DOF robot arm simulator that writes letters on a virtual cardboard using forward/inverse kinematics and dynamics validation.

## Features

- 4-DOF robot arm with real DH parameters
- Forward and inverse kinematics (Robotics Toolbox)
- Trajectory generation for letter writing
- Static and dynamic torque calculations
- Motor feasibility validation
- 3D matplotlib animation
- Configurable via YAML

## Requirements

All requirements met per MCH 4951 specs:
- ✅ 4 independent links (4 DOF)
- ✅ 3D workspace (volumetric)
- ✅ 1 translational joint (prismatic elbow)
- ✅ 2+ rotational joints (base, shoulder, wrist)
- ✅ 1+ offset (shoulder offset)
- ✅ Real materials (aluminum, steel)
- ✅ Real motors (Dynamixel servos)

## Installation

```bash
# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Quick Test (No GUI)
```bash
venv/Scripts/python.exe test_simulation.py
```

### Full Simulation with Animation
```bash
venv/Scripts/python.exe simulator.py
```

### Individual Components

**Test Robot FK/IK:**
```bash
venv/Scripts/python.exe test_robot.py
```

**Test Trajectory Generation:**
```bash
venv/Scripts/python.exe trajectory_generator.py
```

## Project Structure

```
project/
├── config.yaml              # Robot DH parameters, materials, motors
├── robot_designer.py        # Robot class, FK/IK, dynamics
├── trajectory_generator.py  # Letter path generation
├── simulator.py             # Main animation loop
├── test_robot.py            # Test FK/IK
├── test_simulation.py       # Test full simulation
├── requirements.txt         # Python dependencies
└── README.md                # This file
```

## Configuration

Edit `config.yaml` to modify:
- Robot DH parameters (link lengths, offsets, twists)
- Material properties (density, cost)
- Motor specifications (torque, speed limits)
- Letters to write
- Workspace dimensions

## Current Limitations

The current robot configuration has:
1. **High IK errors** (~0.57m average) - robot geometry may need optimization
2. **Motor torque exceedances** - all motors exceed limits, need stronger motors or lighter links

## Optimization Suggestions

To improve feasibility:
1. Increase motor torque ratings (use larger Dynamixel models)
2. Reduce link masses (thinner aluminum tubes)
3. Shorten link lengths for better reachability
4. Adjust cardboard position closer to base

## Output Example

```
Generating trajectory for 'BAU'...
Solving IK for 481 waypoints...

=== Trajectory Analysis ===
Average IK error: 0.5678m
Max IK error: 0.8750m

Torque Analysis:
  base_motor: max 19.52 Nm / 10.00 Nm [EXCEEDED]
  shoulder_motor: max 14.24 Nm / 6.00 Nm [EXCEEDED]
```

## Tech Stack

- **Robotics Toolbox**: Forward/inverse kinematics and dynamics
- **NumPy/SciPy**: Numerical calculations and trajectory interpolation
- **matplotlib**: 3D visualization and animation
- **PyYAML**: Configuration management

## License

Educational project for MCH 4951.
