"""
Interactive Robot Design Visualizer
Single window with controls on left, RTB visualization on right
"""

import numpy as np
import tkinter as tk
from tkinter import ttk
import sys
import os
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# Turn off interactive mode to prevent windows from showing
plt.ioff()

# Load robotics toolbox
from roboticstoolbox.robot.Robot import Robot
import yaml


class RobotVisualizerGUI:
    def __init__(self, config_path='src/config.yaml'):
        # Load config
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)

        # Load robot directly from URDF using RTB
        urdf_path = os.path.join(os.path.dirname(dirname(__file__)), 'config', 'robot.urdf')
        self.robot = Robot.URDF(urdf_path)
        self.robot.gravity = [0, 0, -self.config['simulation']['gravity']]
        self.current_env = None

        # Create main window
        self.root = tk.Tk()
        self.root.title("Robot Design Visualizer")
        self.root.geometry("1400x800")

        # Initialize joint angles (n = number of joints, not links)
        self.num_joints = self.robot.n
        self.current_q = np.zeros(self.num_joints)

        # Set all joints to mid-range
        if self.robot.qlim is not None:
            for i in range(self.num_joints):
                lower = self.robot.qlim[0, i]
                upper = self.robot.qlim[1, i]
                self.current_q[i] = (lower + upper) / 2

        # Create UI
        self.create_widgets()

        # Initial visualization
        self.visualize_robot(self.current_q, "Initial Configuration")

    def create_widgets(self):
        # Main container - split into left (controls) and right (visualization)
        main_container = tk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_container.pack(fill='both', expand=True)

        # LEFT SIDE - Controls
        left_frame = tk.Frame(main_container, width=500)
        main_container.add(left_frame)

        # Title
        title = tk.Label(left_frame, text="Robot Design Controls",
                        font=("Arial", 14, "bold"))
        title.pack(pady=10)

        # Create notebook for tabs
        notebook = ttk.Notebook(left_frame)
        notebook.pack(fill='both', expand=True, padx=10, pady=5)

        # Tab 1: Link Lengths (Design)
        design_frame = ttk.Frame(notebook)
        notebook.add(design_frame, text="Link Lengths")
        self.create_design_sliders(design_frame)

        # Tab 2: Joint Angles (Pose)
        joint_frame = ttk.Frame(notebook)
        notebook.add(joint_frame, text="Joint Angles")
        self.create_joint_sliders(joint_frame)

        # Buttons at bottom of left panel
        button_frame = tk.Frame(left_frame)
        button_frame.pack(pady=10)

        rebuild_btn = tk.Button(
            button_frame,
            text="🔄 Rebuild Robot",
            command=self.rebuild_robot,
            bg='#4CAF50',
            fg='white',
            font=("Arial", 11, "bold"),
            padx=15,
            pady=8
        )
        rebuild_btn.pack(fill='x', padx=5, pady=2)

        viz_btn = tk.Button(
            button_frame,
            text="📐 Update Pose",
            command=self.visualize_pose,
            bg='#2196F3',
            fg='white',
            font=("Arial", 11, "bold"),
            padx=15,
            pady=8
        )
        viz_btn.pack(fill='x', padx=5, pady=2)

        # Status label
        self.status_label = tk.Label(left_frame, text="Ready",
                                     font=("Arial", 9), wraplength=450)
        self.status_label.pack(pady=5)

        # RIGHT SIDE - Visualization
        right_frame = tk.Frame(main_container)
        main_container.add(right_frame)

        # Matplotlib figure container
        viz_title = tk.Label(right_frame, text="Robot Visualization (RTB PyPlot)",
                            font=("Arial", 14, "bold"))
        viz_title.pack(pady=5)

        # Create matplotlib figure placeholder
        self.viz_frame = tk.Frame(right_frame)
        self.viz_frame.pack(fill='both', expand=True, padx=5, pady=5)

    def create_design_sliders(self, parent):
        """Create sliders for modifying link lengths"""
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Design parameters
        self.design_sliders = {}

        # Base height
        base_height_frame = self.create_slider_widget(
            scrollable_frame, "Base Height (m)",
            0.15, 0.05, 0.50, 0.01
        )
        self.design_sliders['base_height'] = base_height_frame['var']

        # Shoulder length
        shoulder_length_frame = self.create_slider_widget(
            scrollable_frame, "Shoulder Length (m)",
            0.30, 0.10, 0.60, 0.01
        )
        self.design_sliders['shoulder_length'] = shoulder_length_frame['var']

        # Elbow length
        elbow_length_frame = self.create_slider_widget(
            scrollable_frame, "Elbow Length (m)",
            0.35, 0.10, 0.60, 0.01
        )
        self.design_sliders['elbow_length'] = elbow_length_frame['var']

        # Wrist base length
        wrist_base_frame = self.create_slider_widget(
            scrollable_frame, "Wrist Base (m)",
            0.10, 0.05, 0.30, 0.01
        )
        self.design_sliders['wrist_base'] = wrist_base_frame['var']

        # Wrist extension max
        wrist_ext_frame = self.create_slider_widget(
            scrollable_frame, "Wrist Max Extension (m)",
            0.20, 0.05, 0.40, 0.01
        )
        self.design_sliders['wrist_extension'] = wrist_ext_frame['var']

    def create_joint_sliders(self, parent):
        # Container
        canvas = tk.Canvas(parent)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)

        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )

        canvas.create_window((0, 0), window=scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Create sliders for each joint
        self.joint_sliders = []

        # Get joint names from URDF links
        joint_names = ['base_joint', 'shoulder_joint', 'elbow_joint', 'wrist_extend_joint']

        for i in range(self.num_joints):
            # Get limits from qlim matrix [2 x n]
            if self.robot.qlim is not None:
                lower = self.robot.qlim[0, i]
                upper = self.robot.qlim[1, i]
            else:
                lower, upper = -3.14, 3.14  # Default limits

            # Check if prismatic (joint 3 is wrist extension)
            is_prismatic = (i == 3)

            if is_prismatic:
                frame = self.create_slider_widget(
                    scrollable_frame, f"{joint_names[i]} (m)",
                    self.current_q[i], lower, upper, 0.01
                )
            else:
                frame = self.create_slider_widget(
                    scrollable_frame, f"{joint_names[i]} (deg)",
                    np.degrees(self.current_q[i]),
                    np.degrees(lower), np.degrees(upper), 1.0
                )

            self.joint_sliders.append({
                'var': frame['var'],
                'is_prismatic': is_prismatic
            })

    def create_slider_widget(self, parent, label, initial, min_val, max_val, resolution):
        """Create a labeled slider widget"""
        frame = tk.Frame(parent)
        frame.pack(fill='x', padx=10, pady=3)

        # Label
        lbl = tk.Label(frame, text=label, width=18, anchor='w', font=("Arial", 9))
        lbl.pack(side='left')

        # Variable
        var = tk.DoubleVar(value=initial)

        # Slider
        slider = tk.Scale(
            frame,
            from_=min_val,
            to=max_val,
            resolution=resolution,
            orient='horizontal',
            variable=var,
            length=200
        )
        slider.pack(side='left', fill='x', expand=True)

        # Value label
        value_lbl = tk.Label(frame, textvariable=var, width=7, font=("Arial", 9))
        value_lbl.pack(side='left')

        return {'var': var, 'slider': slider, 'label': lbl}

    def rebuild_robot(self):
        """Rebuild robot with new link lengths from design sliders"""
        self.status_label.config(text="Rebuilding robot...")
        self.root.update()

        try:
            # Get design parameters from sliders
            base_height = self.design_sliders['base_height'].get()
            shoulder_length = self.design_sliders['shoulder_length'].get()
            elbow_length = self.design_sliders['elbow_length'].get()
            wrist_base = self.design_sliders['wrist_base'].get()
            wrist_extension = self.design_sliders['wrist_extension'].get()

            # Generate new URDF
            self.generate_urdf(base_height, shoulder_length, elbow_length, wrist_base, wrist_extension)

            # Reload robot from URDF using RTB directly
            urdf_path = os.path.join(os.path.dirname(dirname(__file__)), 'config', 'robot.urdf')
            self.robot = Robot.URDF(urdf_path)
            self.robot.gravity = [0, 0, -self.config['simulation']['gravity']]

            # Reset joint angles to mid-range
            self.num_joints = self.robot.n
            self.current_q = np.zeros(self.num_joints)
            if self.robot.qlim is not None:
                for i in range(self.num_joints):
                    lower = self.robot.qlim[0, i]
                    upper = self.robot.qlim[1, i]
                    self.current_q[i] = (lower + upper) / 2

            # Update joint sliders with new values
            for i, slider_dict in enumerate(self.joint_sliders):
                if not slider_dict['is_prismatic']:
                    slider_dict['var'].set(np.degrees(self.current_q[i]))
                else:
                    slider_dict['var'].set(self.current_q[i])

            # Visualize
            self.visualize_robot(self.current_q, "Rebuilt Design")
            self.status_label.config(text="✓ Robot rebuilt successfully!")

        except Exception as e:
            self.status_label.config(text=f"Error: {str(e)}")
            print(f"Error rebuilding robot: {e}")
            import traceback
            traceback.print_exc()

    def generate_urdf(self, base_height, shoulder_length, elbow_length, wrist_base, wrist_extension):
        """Generate URDF file with specified link lengths"""
        urdf_content = f'''<?xml version="1.0"?>
<robot name="letter_writer_4dof">

  <!-- Base Link (fixed to ground) -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder radius="0.03" length="{base_height}"/>
      </geometry>
      <origin xyz="0 0 {base_height/2}" rpy="0 0 0"/>
    </visual>
    <inertial>
      <mass value="4.0"/>
      <origin xyz="0 0 {base_height/2}" rpy="0 0 0"/>
      <inertia ixx="0.703" iyy="0.706" izz="0.009" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint 1: Base Rotation (Revolute - Rz) -->
  <joint name="base_joint" type="revolute">
    <parent link="base_link"/>
    <child link="shoulder_link"/>
    <origin xyz="0 0 {base_height}" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="100.0" velocity="2.0"/>
  </joint>

  <!-- Shoulder Link -->
  <link name="shoulder_link">
    <visual>
      <geometry>
        <cylinder radius="0.02" length="{shoulder_length}"/>
      </geometry>
      <origin xyz="{shoulder_length/2} 0 0" rpy="0 1.5708 0"/>
    </visual>
    <inertial>
      <mass value="2.8"/>
      <origin xyz="{shoulder_length/2} 0 0" rpy="0 0 0"/>
      <inertia ixx="0.21" iyy="0.21" izz="0.0011" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint 2: Shoulder Pitch (Revolute - Ry) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="shoulder_link"/>
    <child link="elbow_link"/>
    <origin xyz="{shoulder_length} 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50.0" velocity="2.0"/>
  </joint>

  <!-- Elbow Link -->
  <link name="elbow_link">
    <visual>
      <geometry>
        <cylinder radius="0.015" length="{elbow_length}"/>
      </geometry>
      <origin xyz="{elbow_length/2} 0 0" rpy="0 1.5708 0"/>
    </visual>
    <inertial>
      <mass value="2.1"/>
      <origin xyz="{elbow_length/2} 0 0" rpy="0 0 0"/>
      <inertia ixx="0.215" iyy="0.215" izz="0.0005" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint 3: Elbow Pitch (Revolute - Ry) -->
  <joint name="elbow_joint" type="revolute">
    <parent link="elbow_link"/>
    <child link="wrist_link"/>
    <origin xyz="{elbow_length} 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.09" upper="0.35" effort="40.0" velocity="2.0"/>
  </joint>

  <!-- Wrist Link (Prismatic Base) -->
  <link name="wrist_link">
    <visual>
      <geometry>
        <cylinder radius="0.01" length="{wrist_base}"/>
      </geometry>
      <origin xyz="{wrist_base/2} 0 0" rpy="0 1.5708 0"/>
    </visual>
    <inertial>
      <mass value="1.3"/>
      <origin xyz="{wrist_base/2} 0 0" rpy="0 0 0"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.00007" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint 4: Wrist Extension (Prismatic) -->
  <joint name="wrist_extend_joint" type="prismatic">
    <parent link="wrist_link"/>
    <child link="end_effector"/>
    <origin xyz="{wrist_base} 0 0" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="0.0" upper="{wrist_extension}" effort="20.0" velocity="0.5"/>
  </joint>

  <!-- End Effector (Pen) -->
  <link name="end_effector">
    <visual>
      <geometry>
        <sphere radius="0.01"/>
      </geometry>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </visual>
    <inertial>
      <mass value="0.07"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.00001" iyy="0.00001" izz="0.00001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

</robot>
'''
        # Write to file
        urdf_path = os.path.join(os.path.dirname(dirname(__file__)), 'config', 'robot.urdf')
        with open(urdf_path, 'w') as f:
            f.write(urdf_content)

    def visualize_pose(self):
        """Visualize robot with current joint angles"""
        self.status_label.config(text="Updating visualization...")
        self.root.update()

        try:
            # Get joint values from sliders
            for i, slider_info in enumerate(self.joint_sliders):
                if slider_info['is_prismatic']:
                    self.current_q[i] = slider_info['var'].get()
                else:
                    self.current_q[i] = np.radians(slider_info['var'].get())

            # Calculate FK using RTB directly
            T = self.robot.fkine(self.current_q)
            pos = T.t

            # Visualize
            self.visualize_robot(self.current_q, "Joint Configuration")

            # Update status
            self.status_label.config(
                text=f"✓ End effector: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]m"
            )

        except Exception as e:
            self.status_label.config(text=f"Error: {str(e)}")
            print(f"Error visualizing pose: {e}")
            import traceback
            traceback.print_exc()

    def visualize_robot(self, q, title):
        """Embed RTB visualization in the right panel"""
        try:
            # Clear previous visualization
            for widget in self.viz_frame.winfo_children():
                widget.destroy()

            # Save current backend
            original_backend = matplotlib.get_backend()

            # Temporarily switch to Agg (non-interactive) to prevent windows
            matplotlib.use('Agg', force=True)

            # Create figure that we'll pass to RTB
            fig = plt.figure(figsize=(8, 7))

            # Use RTB's plot method with our figure
            env = self.robot.plot(
                q,
                backend='pyplot',
                block=False,
                fig=fig  # Pass our figure to RTB
            )

            # Switch back to TkAgg for embedding
            matplotlib.use('TkAgg', force=True)

            # Set title on the figure
            fig.suptitle(
                f'{self.robot.name} - {title}',
                fontsize=12,
                fontweight='bold'
            )

            # Embed the matplotlib figure in tkinter
            canvas = FigureCanvasTkAgg(fig, master=self.viz_frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill='both', expand=True)

            # Add toolbar for zoom/pan/rotate
            toolbar = NavigationToolbar2Tk(canvas, self.viz_frame)
            toolbar.update()
            canvas.get_tk_widget().pack(fill='both', expand=True)

            # Store environment for potential updates
            self.current_env = env

        except Exception as e:
            print(f"Error in visualization: {e}")
            import traceback
            traceback.print_exc()

            error_label = tk.Label(
                self.viz_frame,
                text=f"Visualization error: {str(e)}",
                font=("Arial", 10),
                wraplength=600
            )
            error_label.pack(expand=True)

    def run(self):
        """Start the GUI application"""
        print("Starting Robot Visualizer GUI...")
        print("Single window with controls on left, visualization on right")
        self.root.mainloop()


if __name__ == '__main__':
    app = RobotVisualizerGUI()
    app.run()
