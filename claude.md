# Robot Arm Simulator - Project Plan

## Tech Stack
- **ikpy**: Forward/inverse kinematics
- **NumPy/SciPy**: Dynamics calculations, trajectory interpolation
- **matplotlib**: 3D visualization and animation
- **PyYAML**: Configuration management

## Project Structure
```
project/
├── config.yaml              # Robot DH parameters, materials, motors
├── robot_designer.py        # Robot class, FK/IK, dynamics
├── trajectory_generator.py  # Letter path generation
├── simulator.py             # Main animation loop
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

## Development Phases
1. Config + ikpy chain setup
2. FK/IK validation
3. Letter trajectory generation
4. Dynamics calculation (torques)
5. Motor validation (limits check)
6. Matplotlib animation

## Key Components
- **DH Parameters**: Define robot geometry
- **Material Database**: Aluminum/steel densities, costs
- **Motor Database**: Real servo specs (torque, weight)
- **Letter Paths**: Normalized waypoints scaled to cardboard
- **Torque Validation**: Static (gravity) + dynamic (inertial)

## Running Scripts
Always use venv Python:
```
venv/Scripts/python.exe your_script.py
```
All dependencies installed in venv (ikpy, numpy, scipy, pyyaml, matplotlib)
