# Robot Arm Simulator - Project Instructions

## Tech Stack
- **Robotics Toolbox Python**: Forward/inverse kinematics, dynamics, URDF loading
- **NumPy/SciPy**: Numerical calculations and trajectory interpolation
- **matplotlib**: 3D visualization and animation
- **PyYAML**: Configuration management

## Project Structure
```
project/
├── config/
│   └── robot.urdf           # Robot geometry (URDF standard format)
├── src/
│   ├── config.yaml          # Motors, workspace, writing parameters
│   ├── simulator.py         # Main simulation and IK solver
│   └── trajectory_generator.py  # Letter path generation
├── examples/
│   ├── visualize_trajectory.py
│   ├── compare_trajectory_vs_ik.py
│   └── robot_visualizer_gui.py
└── requirements.txt
```

## Robot Requirements (MCH 4951)
- 4+ DOF (independent links)
- 3D workspace (volume, not plane)
- ≥1 translational joint (not first link)
- ≥2 rotational joints (1 revolute, 1 twist)
- ≥1 offset (not on last link)
- Real materials & motors
- Write 3 letters on 60×120cm cardboard
- Pen retracts between letters
- Calculate: reachability, static/dynamic torques, feasibility

## Key Components

### Robot Definition (config/robot.urdf)
- Link geometry, masses, inertia tensors
- Joint types, axes, limits
- All robot properties in single URDF file

### Application Config (src/config.yaml)
- Motor specifications (torque limits)
- Workspace definition (cardboard position/size)
- Letter writing parameters
- IK and visualization settings

### Trajectory Generation
- Normalized letter paths (0-1 coordinates)
- Scaled to cardboard with margins
- RTB's ctraj for smooth interpolation
- Pen retraction between letters

### IK Solving
- RTB's ikine_LM (Levenberg-Marquardt)
- Position-only constraint (orientation free)
- Angle wrapping for revolute joints
- Previous solution as initial guess

### Torque Calculation
- RTB's pay() method for per-link gravity
- Properly accounts for center of mass
- No custom physics calculations

## Running Scripts
Always use venv Python:
```bash
venv/Scripts/python.exe src/simulator.py
```

All dependencies installed in venv:
- roboticstoolbox-python
- spatialmath-python
- numpy, scipy, matplotlib, pyyaml

## Design Principles
1. **URDF-Driven**: All robot geometry from URDF file
2. **Generic Code**: No hardcoded robot-specific logic
3. **Pure RTB**: Use Robotics Toolbox methods, avoid custom implementations
4. **Single Source of Truth**: Robot properties only in URDF, not duplicated in config

## Code Style
- **No emojis**: Never use emojis in code, comments, or documentation
- **Professional tone**: Clean, technical documentation
- **Concise comments**: Only explain non-obvious logic
