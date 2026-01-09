# Robot Arm Simulator - MCH 4951 Final Project

A Python-based 4-DOF robot arm simulator that writes letters on a 60×120cm virtual cardboard using forward/inverse kinematics and dynamics validation.

## Features

- Generic URDF-based robot loading (works with any URDF robot)
- Forward and inverse kinematics using Robotics Toolbox Python
- Trajectory generation for letter writing with smooth interpolation
- Static torque calculations with per-link gravity analysis
- Motor feasibility validation
- 3D matplotlib animation with real-time visualization
- Fully configurable via YAML

## Project Requirements (MCH 4951)

- 4+ DOF robot (independent links)
- 3D workspace (volumetric, not planar)
- ≥1 translational joint (prismatic wrist extension)
- ≥2 rotational joints (revolute base, shoulder, elbow)
- ≥1 offset (shoulder offset)
- Real materials (aluminum/steel with actual densities)
- Real motors (servo specifications with torque limits)
- Write 3 letters on 60×120cm cardboard
- Pen retracts between letters
- Calculate reachability, static/dynamic torques, feasibility

## Installation

```bash
# Create virtual environment
python -m venv venv

# Activate (Windows)
venv\Scripts\activate

# Activate (Linux/Mac)
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

## Usage

### Run Main Simulation
```bash
venv/Scripts/python.exe src/simulator.py
```

This will:
1. Load the robot from URDF
2. Generate trajectory for "RLN" letters
3. Solve inverse kinematics for all waypoints
4. Calculate torques at each configuration
5. Display 3D animation with pen trace
6. Show real-time torque plots

### Test Components Individually

**Test trajectory generation:**
```bash
venv/Scripts/python.exe src/trajectory_generator.py
```

**Visualize trajectory (debug tool):**
```bash
venv/Scripts/python.exe examples/visualize_trajectory.py
```

**Compare target vs reached positions:**
```bash
venv/Scripts/python.exe examples/compare_trajectory_vs_ik.py
```

**Interactive robot GUI:**
```bash
venv/Scripts/python.exe examples/robot_visualizer_gui.py
```

## Project Structure

```
MCH_4951_Final_Project/
├── config/
│   └── robot.urdf              # Robot geometry, masses, inertia, joint limits
├── src/
│   ├── config.yaml             # Motor specs, workspace, writing params
│   ├── simulator.py            # Main simulation loop and IK solver
│   └── trajectory_generator.py # Letter path generation
├── examples/
│   ├── visualize_trajectory.py # Debug: view generated waypoints
│   ├── compare_trajectory_vs_ik.py # Debug: compare target vs reached
│   └── robot_visualizer_gui.py # Interactive robot control GUI
├── requirements.txt            # Python dependencies
├── CLAUDE.md                   # Development instructions
└── README.md                   # This file
```

## Configuration

### Robot Configuration (config/robot.urdf)
Define robot geometry using standard URDF format:
- Link lengths, masses, inertia tensors
- Joint types (revolute, prismatic), axes, limits
- Material properties

### Application Configuration (src/config.yaml)
- Motor torque specifications
- Workspace dimensions and cardboard position
- Letter parameters (height, spacing, margins)
- IK settings (mask, max joint step)
- Visualization settings

## Current Performance

- **IK Success Rate**: 100% (111/111 waypoints)
- **Average IK Error**: 0.0000m
- **Letters**: 48cm tall on 60×120cm cardboard
- **Torque Requirements**:
  - Base joint: 0.00 Nm (no gravity effect on vertical axis)
  - Shoulder joint: ~56 Nm (max)
  - Elbow joint: ~50 Nm (max)
  - Wrist extension: ~39 Nm (max)

## Technical Details

### Robotics Toolbox Integration
- Robot loaded via `Robot.URDF()`
- Forward kinematics: `robot.fkine(q)`
- Inverse kinematics: `robot.ikine_LM(T, q0, mask)`
- Gravity torques: `robot.pay(W, q, frame)` per-link
- Cartesian interpolation: `rtb.ctraj(T1, T2, t)`

### Trajectory Generation
1. Generate letter paths (normalized 0-1 coordinates)
2. Scale to cardboard dimensions with margins
3. Add pen retraction moves between letters
4. Interpolate with quintic polynomials (smooth velocity/acceleration)

### IK Solving
- Position-only constraint (orientation free)
- Uses previous solution as initial guess for continuity
- Handles angle wrapping for revolute joints
- Detects and smooths large joint jumps

## Tech Stack

- **Robotics Toolbox Python**: Kinematics, dynamics, URDF parsing
- **Spatial Math Python**: SE3 transforms and rotations
- **NumPy/SciPy**: Numerical calculations
- **Matplotlib**: 3D visualization and animation
- **PyYAML**: Configuration file parsing

## License

Educational project for MCH 4951 Robotics.
