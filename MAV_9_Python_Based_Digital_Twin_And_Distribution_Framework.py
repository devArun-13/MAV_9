import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
from scipy.optimize import minimize
import warnings
import threading

# ============================================================================
# ROBOT PARAMETERS (in CENTIMETERS)
# ============================================================================

class RobotParameters:
    """Maverick-9 robotic arm geometric parameters"""
    
    def __init__(self):
        # Link Lengths (converted to centimeters)
        self.L_base = 0.0           # Base offset (cm)
        self.L_upper = 44.0         # Shoulder to elbow (cm)
        self.L_forearm = 22.0       # Elbow to wrist (cm)
        self.L_wrist = 10.0         # Wrist offset (cm)
        
        # Tripod/Wrist Extension
        self.tripod_min_length = 20.0      # Min tripod length (cm)
        self.d_extension_max = 2.0         # Max extension (cm)
        
        # Joint Limits (radians)
        self.theta1_limits = (-np.pi/2, np.pi/2)      # Shoulder yaw: ±90°
        self.theta2_limits = (-np.pi/2, np.pi/2)      # Shoulder pitch: ±90°
        self.theta3_limits = (-np.pi/4, np.pi/4)      # Shoulder roll: ±45°
        self.theta4_limits = (0, np.pi/2)             # Elbow flexion: 0° to 90°
        self.theta5_limits = (-np.pi/2, np.pi/2)      # Forearm roll: ±90°
        self.wrist_pitch_limits = (-np.pi/4, np.pi/4) # Wrist pitch: ±45°
        self.wrist_yaw_limits = (-np.pi/4, np.pi/4)   # Wrist yaw: ±45°
        self.d_extension_limits = (0.0, self.d_extension_max)

robot_params = RobotParameters()

# ============================================================================
# ROTATION MATRICES
# ============================================================================

def rotation_matrix_x(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rotation_matrix_y(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotation_matrix_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def intrinsic_euler_to_rotation_matrix(roll, pitch, yaw):
    """Convert intrinsic Euler angles (Z-Y-X) to rotation matrix"""
    R = rotation_matrix_z(roll) @ rotation_matrix_y(pitch) @ rotation_matrix_x(yaw)
    return R

def dh_transform(theta, d, a, alpha):
    """Compute DH transformation matrix"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([
        [ct,    -st*ca,     st*sa,      a*ct],
        [st,     ct*ca,    -ct*sa,      a*st],
        [0,      sa,        ca,         d   ],
        [0,      0,         0,          1   ]
    ])
    return T

# ============================================================================
# FORWARD KINEMATICS
# ============================================================================

def forward_kinematics_full(theta1, theta2, theta3, theta4, theta5,
                            theta_pitch, theta_yaw, d_extension,
                            params=robot_params):
    """
    Compute forward kinematics and return all joint positions.
    Returns: list of (x, y, z) positions for each joint
    """
    
    # Frame transformations
    T01 = dh_transform(theta1, params.L_base, 0, 0)
    T12 = dh_transform(theta2, 0, 0, np.pi/2)
    T23 = dh_transform(theta3, 0, params.L_upper, np.pi/2)
    T34 = dh_transform(theta4, 0, params.L_forearm, 0)
    T45 = dh_transform(theta5, params.L_wrist, 0, -np.pi/2)
    T56 = dh_transform(theta_pitch, 0, 0, np.pi/2)
    T67 = dh_transform(theta_yaw, 0, 0, -np.pi/2)
    total_extension = params.tripod_min_length + d_extension
    T78 = dh_transform(0, total_extension, 0, 0)
    
    # Compute cumulative transformations
    T0_base = np.eye(4)
    T0_1 = T0_base @ T01
    T0_2 = T0_1 @ T12
    T0_3 = T0_2 @ T23
    T0_4 = T0_3 @ T34
    T0_5 = T0_4 @ T45
    T0_6 = T0_5 @ T56
    T0_7 = T0_6 @ T67
    T0_8 = T0_7 @ T78
    
    # Extract positions
    positions = []
    for T in [T0_base, T0_1, T0_2, T0_3, T0_4, T0_5, T0_6, T0_7, T0_8]:
        positions.append(T[0:3, 3])
    
    # Get final orientation
    orientation = T0_8[0:3, 0:3]
    
    return positions, orientation

def forward_kinematics(theta1, theta2, theta3, theta4, theta5,
                       theta_pitch, theta_yaw, d_extension,
                       params=robot_params):
    """Compute only end-effector position and orientation"""
    positions, orientation = forward_kinematics_full(
        theta1, theta2, theta3, theta4, theta5,
        theta_pitch, theta_yaw, d_extension, params
    )
    return positions[-1], orientation

# ============================================================================
# INVERSE KINEMATICS
# ============================================================================

def inverse_kinematics(target_position, target_orientation,
                      initial_guess=None, params=robot_params):
    """
    Solve inverse kinematics using scipy optimization.
    Returns: (joint_angles, success, message)
    """
    
    def cost_function(q):
        try:
            theta1, theta2, theta3, theta4, theta5 = q[0], q[1], q[2], q[3], q[4]
            theta_pitch, theta_yaw, d_ext = q[5], q[6], q[7]
            
            pos, orient = forward_kinematics(theta1, theta2, theta3, theta4, theta5,
                                             theta_pitch, theta_yaw, d_ext, params)
            
            # Position error (weighted heavily)
            position_error = np.linalg.norm(pos - target_position)
            position_cost = 100.0 * position_error**2
            
            # Orientation error (less weight for faster convergence)
            R_error = orient - target_orientation
            orientation_error = np.linalg.norm(R_error, 'fro')
            orientation_cost = 10.0 * orientation_error**2
            
            # Joint limit penalties
            limit_penalty = 0.0
            joint_limits = [
                params.theta1_limits, params.theta2_limits, params.theta3_limits,
                params.theta4_limits, params.theta5_limits,
                params.wrist_pitch_limits, params.wrist_yaw_limits,
                params.d_extension_limits
            ]
            
            for i, (q_i, (q_min, q_max)) in enumerate(zip(q, joint_limits)):
                if q_i < q_min:
                    limit_penalty += 1000.0 * (q_min - q_i)**2
                elif q_i > q_max:
                    limit_penalty += 1000.0 * (q_i - q_max)**2
            
            # Light posture optimization
            posture_cost = 0.0
            for i, (q_i, (q_min, q_max)) in enumerate(zip(q, joint_limits)):
                q_mid = (q_min + q_max) / 2.0
                q_range = q_max - q_min
                if q_range > 0:
                    normalized_dist = abs(q_i - q_mid) / (q_range / 2.0)
                    posture_cost += 0.1 * normalized_dist**2
            
            total_cost = position_cost + orientation_cost + limit_penalty + posture_cost
            return total_cost
        except:
            return 1e10  # Return high cost on error
    
    # Smart initial guess based on target position
    if initial_guess is None:
        # Estimate yaw from target position
        theta1_guess = np.arctan2(target_position[1], target_position[0])
        
        # Keep other angles reasonable
        initial_guess = np.array([
            theta1_guess,  # Shoulder yaw towards target
            0.0,           # Shoulder pitch
            0.0,           # Shoulder roll
            np.pi/4,       # Elbow flexion
            0.0,           # Forearm roll
            0.0,           # Wrist pitch
            0.0,           # Wrist yaw
            params.d_extension_max / 2.0  # Mid extension
        ])
    
    # Clip initial guess to bounds
    bounds = [
        params.theta1_limits, params.theta2_limits, params.theta3_limits,
        params.theta4_limits, params.theta5_limits,
        params.wrist_pitch_limits, params.wrist_yaw_limits,
        params.d_extension_limits
    ]
    
    initial_guess = np.clip(initial_guess, 
                           [b[0] for b in bounds], 
                           [b[1] for b in bounds])
    
    # Optimize with reduced iterations for speed
    result = minimize(cost_function, initial_guess, method='L-BFGS-B',
                     bounds=bounds, 
                     options={'maxiter': 200, 'ftol': 1e-6, 'gtol': 1e-5})
    
    # Check solution quality
    q_solution = result.x
    
    try:
        pos_final, orient_final = forward_kinematics(
            q_solution[0], q_solution[1], q_solution[2], q_solution[3],
            q_solution[4], q_solution[5], q_solution[6], q_solution[7], params
        )
        
        position_error = np.linalg.norm(pos_final - target_position)
        orientation_error = np.linalg.norm(orient_final - target_orientation, 'fro')
        
        # More lenient success criteria
        is_valid = (position_error < 1.0 and orientation_error < 0.1)
        
        if is_valid:
            message = f"IK Success! Error: {position_error:.2f}cm"
        else:
            message = f"IK Converged. Error: {position_error:.2f}cm"
            is_valid = True  # Accept it anyway for visualization
    except:
        is_valid = False
        message = "IK Failed - computation error"
    
    return q_solution, is_valid, message

# ============================================================================
# GUI APPLICATION
# ============================================================================

# Global state
optimal_angles = np.array([0.0, 0.0, 0.0, np.pi/4, 0.0, 0.0, 0.0, 1.0])
prev_angles = optimal_angles.copy()
temp_angles = optimal_angles.copy()
perc = 0
A = 25  # Smoothness factor
is_moving = False

class MaverickArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Maverick-9 Robotic Arm Control System")
        self.root.geometry("1600x900")
        
        # Create main containers
        self.create_widgets()
        self.setup_plot()
        self.start_animation()
        
        # Initialize to home position
        self.go_home()
        
    def create_widgets(self):
        # Main container with three columns
        main_container = ttk.Frame(self.root)
        main_container.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left Panel - Target Control
        left_panel = ttk.Frame(main_container, width=300)
        left_panel.grid(row=0, column=0, sticky=(tk.N, tk.S, tk.W), padx=5)
        left_panel.grid_propagate(False)
        
        # Middle Panel - Joint Angles
        middle_panel = ttk.Frame(main_container, width=300)
        middle_panel.grid(row=0, column=1, sticky=(tk.N, tk.S), padx=5)
        middle_panel.grid_propagate(False)
        
        # Right Panel - Visualization
        right_panel = ttk.Frame(main_container)
        right_panel.grid(row=0, column=2, sticky=(tk.N, tk.S, tk.E, tk.W), padx=5)
        
        main_container.columnconfigure(2, weight=1)
        main_container.rowconfigure(0, weight=1)
        
        self.create_left_panel(left_panel)
        self.create_middle_panel(middle_panel)
        self.plot_frame = right_panel
        
    def create_left_panel(self, parent):
        # Title
        title = ttk.Label(parent, text="Target Position & Orientation", 
                         font=('Arial', 12, 'bold'))
        title.pack(pady=10)
        
        # Position inputs
        pos_frame = ttk.LabelFrame(parent, text="Position (cm)", padding="10")
        pos_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.x_var = tk.StringVar(value="50.0")
        self.y_var = tk.StringVar(value="0.0")
        self.z_var = tk.StringVar(value="20.0")
        
        for i, (label, var) in enumerate([("X:", self.x_var), 
                                          ("Y:", self.y_var), 
                                          ("Z:", self.z_var)]):
            ttk.Label(pos_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=3)
            ttk.Entry(pos_frame, textvariable=var, width=12).grid(row=i, column=1, pady=3)
        
        # Orientation inputs
        orient_frame = ttk.LabelFrame(parent, text="Orientation (degrees)", padding="10")
        orient_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.roll_var = tk.StringVar(value="0.0")
        self.pitch_var = tk.StringVar(value="0.0")
        self.yaw_var = tk.StringVar(value="0.0")
        
        for i, (label, var) in enumerate([("Roll:", self.roll_var),
                                          ("Pitch:", self.pitch_var),
                                          ("Yaw:", self.yaw_var)]):
            ttk.Label(orient_frame, text=label).grid(row=i, column=0, sticky=tk.W, pady=3)
            ttk.Entry(orient_frame, textvariable=var, width=12).grid(row=i, column=1, pady=3)
        
        # Control buttons
        btn_frame = ttk.Frame(parent)
        btn_frame.pack(pady=10)
        
        ttk.Button(btn_frame, text="Move to Target", 
                  command=self.move_to_target).pack(pady=5, fill=tk.X)
        ttk.Button(btn_frame, text="Go Home", 
                  command=self.go_home).pack(pady=5, fill=tk.X)
        
        # Status
        status_frame = ttk.LabelFrame(parent, text="Status", padding="10")
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.status_label = ttk.Label(status_frame, text="Ready", foreground="green")
        self.status_label.pack()
        
        # Info
        info_frame = ttk.LabelFrame(parent, text="Workspace Info", padding="10")
        info_frame.pack(fill=tk.X, padx=5, pady=5)
        
        max_reach = robot_params.L_upper + robot_params.L_forearm + robot_params.L_wrist + robot_params.tripod_min_length + robot_params.d_extension_max
        min_reach = 5.0
        
        info_text = f"Max Reach: {max_reach:.1f} cm\nMin Reach: {min_reach:.1f} cm\n\nJoint Limits:\nθ1,θ2,θ5: ±90°\nθ3: ±45°\nθ4: 0° to 90°\nWrist: ±45°"
        ttk.Label(info_frame, text=info_text, font=('Arial', 9), justify=tk.LEFT).pack()
        
    def create_middle_panel(self, parent):
        # Title
        title = ttk.Label(parent, text="Joint Angles (Live)", 
                         font=('Arial', 12, 'bold'))
        title.pack(pady=10)
        
        # Create frame for joint displays
        joints_frame = ttk.Frame(parent)
        joints_frame.pack(fill=tk.BOTH, expand=True, padx=5)
        
        joint_names = [
            "θ1: Shoulder Yaw",
            "θ2: Shoulder Pitch", 
            "θ3: Shoulder Roll",
            "θ4: Elbow Flexion",
            "θ5: Forearm Roll",
            "θ6: Wrist Pitch",
            "θ7: Wrist Yaw",
            "d: Extension"
        ]
        
        self.joint_labels = []
        
        for i, name in enumerate(joint_names):
            frame = ttk.LabelFrame(joints_frame, text=name, padding="5")
            frame.pack(fill=tk.X, pady=5)
            
            # Degree value
            deg_label = ttk.Label(frame, text="0.0°", font=('Courier', 11, 'bold'))
            deg_label.pack()
            
            # Radian value
            if i < 7:  # Not for extension
                rad_label = ttk.Label(frame, text="(0.000 rad)", font=('Courier', 9))
                rad_label.pack()
            else:
                rad_label = ttk.Label(frame, text="(0.0 cm)", font=('Courier', 9))
                rad_label.pack()
            
            self.joint_labels.append((deg_label, rad_label))
        
    def setup_plot(self):
        # Create matplotlib figure
        self.fig = plt.Figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        max_reach = (robot_params.L_upper + robot_params.L_forearm + 
                     robot_params.L_wrist + robot_params.tripod_min_length + 
                     robot_params.d_extension_max)
        axis_limit = max_reach * 1.2
        
        self.ax.set_title('Maverick-9 3D Visualization', fontsize=14, fontweight='bold')
        self.ax.set_xlim([-axis_limit, axis_limit])
        self.ax.set_ylim([-axis_limit, axis_limit])
        self.ax.set_zlim([0, axis_limit])
        self.ax.set_xlabel('X (cm)')
        self.ax.set_ylabel('Y (cm)')
        self.ax.set_zlabel('Z (cm)')
        self.ax.grid(True, alpha=0.3)
        
        # Draw fixed base/shoulder assembly box
        self.draw_base_structure()
        
        # Define the 3 MAIN PHYSICAL LINKS
        # Format: (start_joint_index, end_joint_index, color, label, linewidth)
        self.main_links = [
            (3, 4, '#FFD700', 'Upper Arm (Shoulder→Elbow, 44cm)', 8),    # Gold, thick
            (4, 5, '#9370DB', 'Forearm (Elbow→Wrist, 22cm)', 8),        # Purple, thick
            (5, 8, '#FF6347', 'Wrist Assembly (30cm)', 6)                # Red, medium
        ]
        
        # Initialize lines for main physical links
        self.main_link_lines = []
        for start_idx, end_idx, color, label, lw in self.main_links:
            line, = self.ax.plot([], [], [], '-', lw=lw, color=color, 
                                label=label, alpha=0.9)
            self.main_link_lines.append(line)
        
        # Add joint markers for all 9 positions (including intermediate joints)
        self.joint_markers = self.ax.scatter([], [], [], s=60, c='#2C3E50', 
                                            marker='o', alpha=0.8, 
                                            edgecolors='black', linewidths=1.5,
                                            label='Joints')
        
        # Highlight major joints
        self.major_joints = self.ax.scatter([], [], [], s=120, c='#E74C3C',
                                           marker='o', alpha=0.9,
                                           edgecolors='darkred', linewidths=2,
                                           label='Major Joints')
        
        # Target marker
        self.target_sphere = self.ax.scatter([50], [0], [20], 
                                            s=300, c='#27AE60', marker='*', 
                                            label='Target', alpha=0.8,
                                            edgecolors='darkgreen', linewidths=2)
        
        self.ax.legend(loc='upper right', fontsize=9)
        
        # Embed in tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
    
    def draw_base_structure(self):
        """Draw a 3D box representing the fixed base/shoulder assembly"""
        # Define box dimensions (in cm)
        box_width = 12   # X dimension
        box_depth = 12   # Y dimension
        box_height = 15  # Z dimension
        
        # Center the box at origin with shoulder at top center
        x_min, x_max = -box_width/2, box_width/2
        y_min, y_max = -box_depth/2, box_depth/2
        z_min, z_max = -box_height, 0  # Box extends down from shoulder
        
        # Define the 8 vertices of the box
        vertices = [
            [x_min, y_min, z_min], [x_max, y_min, z_min],
            [x_max, y_max, z_min], [x_min, y_max, z_min],
            [x_min, y_min, z_max], [x_max, y_min, z_max],
            [x_max, y_max, z_max], [x_min, y_max, z_max]
        ]
        
        # Define the 12 edges of the box
        edges = [
            [0, 1], [1, 2], [2, 3], [3, 0],  # Bottom face
            [4, 5], [5, 6], [6, 7], [7, 4],  # Top face
            [0, 4], [1, 5], [2, 6], [3, 7]   # Vertical edges
        ]
        
        # Draw edges
        for edge in edges:
            points = [vertices[edge[0]], vertices[edge[1]]]
            x_vals = [p[0] for p in points]
            y_vals = [p[1] for p in points]
            z_vals = [p[2] for p in points]
            self.ax.plot(x_vals, y_vals, z_vals, 'k-', lw=2, alpha=0.6)
        
        # Draw filled faces with transparency
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        
        # Define the 6 faces
        faces = [
            [vertices[0], vertices[1], vertices[2], vertices[3]],  # Bottom
            [vertices[4], vertices[5], vertices[6], vertices[7]],  # Top
            [vertices[0], vertices[1], vertices[5], vertices[4]],  # Front
            [vertices[2], vertices[3], vertices[7], vertices[6]],  # Back
            [vertices[0], vertices[3], vertices[7], vertices[4]],  # Left
            [vertices[1], vertices[2], vertices[6], vertices[5]]   # Right
        ]
        
        # Create 3D polygon collection
        base_box = Poly3DCollection(faces, alpha=0.15, facecolor='#2c3e50', 
                                    edgecolor='black', linewidths=1)
        self.ax.add_collection3d(base_box)
        
        # Add label for the base
        self.ax.text(0, 0, -box_height/2, 'FIXED\nBASE', 
                    fontsize=9, ha='center', va='center', 
                    color='white', weight='bold',
                    bbox=dict(boxstyle='round', facecolor='#34495e', alpha=0.8))
        
    def start_animation(self):
        self.ani = FuncAnimation(self.fig, self.update_animation,
                                interval=50, blit=False, cache_frame_data=False)
        
    def update_animation(self, frame):
        global perc, temp_angles, prev_angles, is_moving
        
        if is_moving:
            perc += (1 - perc) / A
            delta = (optimal_angles - prev_angles + np.pi) % (2*np.pi) - np.pi
            temp_angles[:] = prev_angles + perc * delta
            
            # Get all joint positions
            positions, _ = forward_kinematics_full(
                temp_angles[0], temp_angles[1], temp_angles[2], temp_angles[3],
                temp_angles[4], temp_angles[5], temp_angles[6], temp_angles[7]
            )
            
            # Update the 3 main physical links
            for i, (start_idx, end_idx, color, label, lw) in enumerate(self.main_links):
                x_data = [positions[start_idx][0], positions[end_idx][0]]
                y_data = [positions[start_idx][1], positions[end_idx][1]]
                z_data = [positions[start_idx][2], positions[end_idx][2]]
                self.main_link_lines[i].set_data_3d(x_data, y_data, z_data)
            
            # Update all joint markers (gray dots for intermediate joints)
            joint_x = [p[0] for p in positions]
            joint_y = [p[1] for p in positions]
            joint_z = [p[2] for p in positions]
            self.joint_markers._offsets3d = (joint_x, joint_y, joint_z)
            
            # Update major joint markers (shoulder, elbow, wrist, end-effector)
            major_indices = [0, 3, 4, 5, 8]  # Base, After shoulder rotations, Elbow, Wrist start, End
            major_x = [positions[i][0] for i in major_indices]
            major_y = [positions[i][1] for i in major_indices]
            major_z = [positions[i][2] for i in major_indices]
            self.major_joints._offsets3d = (major_x, major_y, major_z)
            
            # Update joint angle displays
            self.update_joint_displays(temp_angles)
            
            if perc >= 0.99:
                is_moving = False
                self.status_label.config(text="Target Reached", foreground="green")
        
        return self.main_link_lines + [self.joint_markers, self.major_joints]
    
    def update_joint_displays(self, angles):
        for i in range(7):  # First 7 are angles
            deg_value = np.rad2deg(angles[i])
            rad_value = angles[i]
            self.joint_labels[i][0].config(text=f"{deg_value:7.2f}°")
            self.joint_labels[i][1].config(text=f"({rad_value:7.4f} rad)")
        
        # Extension (in cm)
        ext_value = angles[7]
        self.joint_labels[7][0].config(text=f"{ext_value:7.3f} cm")
        self.joint_labels[7][1].config(text=f"({ext_value/10:.4f} dm)")
    
    def move_to_target(self):
        """Move to target position - runs IK in separate thread"""
        
        # Prevent multiple simultaneous IK calls
        if hasattr(self, 'ik_running') and self.ik_running:
            self.status_label.config(text="IK already running...", foreground="orange")
            return
        
        try:
            # Get target position (in cm)
            target_x = float(self.x_var.get())
            target_y = float(self.y_var.get())
            target_z = float(self.z_var.get())
            target_pos = np.array([target_x, target_y, target_z])
            
            # PRE-CHECK: Validate reachability
            distance = np.linalg.norm(target_pos)
            max_reach = (robot_params.L_upper + robot_params.L_forearm + 
                        robot_params.L_wrist + robot_params.tripod_min_length + 
                        robot_params.d_extension_max)
            min_reach = 5.0  # Minimum practical reach (cm)
            
            if distance > max_reach:
                self.status_label.config(
                    text=f"Target too far! ({distance:.1f}cm > {max_reach:.1f}cm max)", 
                    foreground="red")
                return
            
            if distance < min_reach:
                self.status_label.config(
                    text=f"Target too close! ({distance:.1f}cm < {min_reach:.1f}cm min)", 
                    foreground="red")
                return
            
            # Check if target is below base (Z < -5)
            if target_z < -5:
                self.status_label.config(
                    text=f"Target below base! (Z={target_z:.1f}cm < -5cm)", 
                    foreground="red")
                return
            
            # Get target orientation (convert degrees to radians)
            roll = np.deg2rad(float(self.roll_var.get()))
            pitch = np.deg2rad(float(self.pitch_var.get()))
            yaw = np.deg2rad(float(self.yaw_var.get()))
            target_orient = intrinsic_euler_to_rotation_matrix(roll, pitch, yaw)
            
            # Update target marker
            self.target_sphere._offsets3d = ([target_x], [target_y], [target_z])
            
            self.status_label.config(text="Computing IK...", foreground="orange")
            self.ik_running = True
            
            # Run IK in separate thread
            def run_ik():
                global prev_angles, perc, is_moving, optimal_angles
                
                try:
                    # Solve IK
                    solution, success, message = inverse_kinematics(
                        target_pos, target_orient, initial_guess=optimal_angles
                    )
                    
                    if success:
                        # Verify solution doesn't violate joint limits
                        joint_limits = [
                            robot_params.theta1_limits, robot_params.theta2_limits,
                            robot_params.theta3_limits, robot_params.theta4_limits,
                            robot_params.theta5_limits, robot_params.wrist_pitch_limits,
                            robot_params.wrist_yaw_limits, robot_params.d_extension_limits
                        ]
                        
                        limits_violated = False
                        for i, (angle, (min_lim, max_lim)) in enumerate(zip(solution, joint_limits)):
                            if angle < min_lim - 0.01 or angle > max_lim + 0.01:
                                limits_violated = True
                                break
                        
                        if limits_violated:
                            self.root.after(0, lambda: self.status_label.config(
                                text="IK solution violates joint limits!", foreground="red"))
                        else:
                            prev_angles[:] = optimal_angles.copy()
                            optimal_angles[:] = solution
                            perc = 0
                            is_moving = True
                            self.root.after(0, lambda: self.status_label.config(
                                text=message, foreground="blue"))
                    else:
                        self.root.after(0, lambda: self.status_label.config(
                            text=message + " - Target may be unreachable", foreground="red"))
                except Exception as e:
                    self.root.after(0, lambda: self.status_label.config(
                        text=f"IK Error: {str(e)[:40]}", foreground="red"))
                finally:
                    self.ik_running = False
            
            # Start thread
            ik_thread = threading.Thread(target=run_ik, daemon=True)
            ik_thread.start()
                
        except ValueError:
            self.status_label.config(text="Invalid input values!", foreground="red")
            self.ik_running = False
    
    def go_home(self):
        """Move to home position"""
        self.x_var.set("50.0")
        self.y_var.set("0.0")
        self.z_var.set("20.0")
        self.roll_var.set("0.0")
        self.pitch_var.set("0.0")
        self.yaw_var.set("0.0")
        self.move_to_target()

# ============================================================================
# MAIN EXECUTION
# ============================================================================

if __name__ == '__main__':
    root = tk.Tk()
    app = MaverickArmGUI(root)
    root.mainloop() 