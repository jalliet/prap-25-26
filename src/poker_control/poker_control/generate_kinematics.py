import casadi as ca
import os
import numpy as np
from ament_index_python.packages import get_package_share_directory

def main():
    print("Generating Poker Arm Kinematics (FK, IK, Jacobian)...")
    
    # --- 1. Robot Definition (Geometry) ---
    x_0, y_0 = 0.5, 0.2
    L = {
        'l1': 0.07, 'l2': 0.07, 'l3': 0.03, 'l4': 0.115,
        'l5': 0.03, 'l6': 0.137, 'l7': 0.0045, 'l8': 0.06,
        'l9': 0.03, 'l10': 0.02, 'l11': 0.055, 'l12': 0.055
    }
    
    # DH Table [d, theta_offset, a, alpha]
    dh_params = [
        [0, 0, x_0, 0],              # 0: WtoP1
        [L['l1'], ca.pi/2, y_0, 0],  # 1: P1toB (Base Frame)
        [0, 0, 0, 0],                # 2: Bto1 (Joint 1)
        [L['l2'], 0, L['l3'], -ca.pi/2], # 3: 1toP2
        [0, 0, L['l4'], 0],          # 4: P2to2 (Joint 2)
        [0, -ca.pi/2, L['l5'], 0],   # 5: 2toP3
        [0, ca.pi/2, L['l6'], 0],    # 6: P3to3 (Joint 3)
        [0, -ca.pi/2, L['l7'], 0],   # 7: 3toP4
        [0, ca.pi/2, L['l8'], 0],    # 8: P4to4 (Joint 4 - Wrist Center)
        [0, -ca.pi/2, 0, -ca.pi/2],  # 9: 4toP5
        [0, 0, 0, 0],                # 10: P5to5 (Joint 5 - Pitch)
        [L['l9'], 0, L['l10'], ca.pi/2], # 11: 5toP6
        [0, ca.pi/2, L['l11'], 0],   # 12: P6to6 (Joint 6 - Roll)
        [0, ca.pi/2, L['l12'], 0],   # 13: 6toalign1
        [0, -ca.pi/2, 0, ca.pi/2],   # 14: align1toalign2
        [0, -ca.pi/2, 0, 0]          # 15: align2toEE
    ]
    
    # Indices in dh_params that are controllable joints
    joint_indices = [2, 4, 6, 8, 10, 12] # q1, q2, q3, q4, q5, q6

    # --- 2. Helper: DH Matrix Function ---
    def get_dh_matrix(d, theta, a, alpha):
        ct, st = ca.cos(theta), ca.sin(theta)
        ca_alpha, sa_alpha = ca.cos(alpha), ca.sin(alpha)
        return ca.vertcat(
            ca.horzcat(ct, -st*ca_alpha,  st*sa_alpha, a*ct),
            ca.horzcat(st,  ct*ca_alpha, -ct*sa_alpha, a*st),
            ca.horzcat(0, sa_alpha, ca_alpha, d),
            ca.horzcat(0, 0, 0, 1)
        )

    # --- 3. Forward Kinematics (Symbolic) ---
    q = ca.SX.sym('q', 6)
    
    # Initialize with symbolic q to track joint positions and axes
    H = ca.SX.eye(4)
    H_init = ca.DM.eye(4)
    
    joint_positions = []
    joint_rotation_axes = []
    init_joint_positions = []
    
    base_to_world = None
    world_to_base = None
    frame_2_x_axis = None
    frame_3_x_axis = None
    joint_3_R = None
    
    joint_id = 0
    for i, row in enumerate(dh_params):
        d, theta_offset, a, alpha = row[0], row[1], row[2], row[3]
        theta = theta_offset
        
        # Add joint angle if this is a joint
        if joint_id < 6 and i == joint_indices[joint_id]:
            theta = q[joint_id] + theta_offset
            
            # Store joint information
            init_joint_positions.append(H_init[0:3, 3])
            joint_positions.append(H[0:3, 3])
            joint_rotation_axes.append(H[0:3, 2])  # z-axis
            
            if joint_id == 0:
                base_to_world = H
                R_BW = H[0:3, 0:3]
                p_BW = H[0:3, 3]
                world_to_base = ca.vertcat(
                    ca.horzcat(R_BW.T, -R_BW.T @ p_BW),
                    ca.horzcat(0, 0, 0, 1)
                )
            if joint_id == 1:
                frame_2_x_axis = H_init[0:3, 0]
            if joint_id == 2:
                frame_3_x_axis = H_init[0:3, 0]
            if joint_id == 3:
                joint_3_R = H[0:3, 0:3]
                
            joint_id += 1
        
        # Compute transformation matrices
        H_i = get_dh_matrix(d, theta, a, alpha)
        H_init_i = get_dh_matrix(d, theta_offset, a, alpha)
        
        H = H @ H_i
        H_init = H_init @ H_init_i
    
    # Extract EE Pose
    pos_ee = H[0:3, 3]
    rot_ee = H[0:3, 0:3]
    
    # Euler angles (roll, pitch, yaw)
    roll_fk = ca.atan2(rot_ee[2,1], rot_ee[2,2]) 
    pitch_fk = ca.asin(-rot_ee[2,0])
    yaw_fk = ca.atan2(rot_ee[1,0], rot_ee[0,0])
    
    fk_out = ca.vertcat(pos_ee, roll_fk, pitch_fk, yaw_fk)
    fk_func = ca.Function('FK_func', [q], [fk_out])
    
    # Helper to get Joint 2 Position (for IK)
    joint_2_pos_func = ca.Function('joint_2_pos_func', [q[0]], [joint_positions[1]])

    # Helper for World to Wrist Orientation 
    world_to_wrist_func = ca.Function('world_to_wrist_func', [q[0:3]], [joint_3_R])

    # --- 4. Jacobian (Symbolic) ---
    def skew_matrix(axes):
        return ca.vertcat(
            ca.horzcat(0, -axes[2], axes[1]),
            ca.horzcat(axes[2], 0, -axes[0]),
            ca.horzcat(-axes[1], axes[0], 0)
        )
    
    J_p = ca.SX.zeros(3, 6)
    J_o = ca.SX.zeros(3, 6)
    
    for i in range(6):
        J_p[:, i] = skew_matrix(joint_rotation_axes[i]) @ (pos_ee - joint_positions[i])
        J_o[:, i] = joint_rotation_axes[i]
        
    jacobian = ca.vertcat(J_p, J_o)
    jacobian_func = ca.Function('jacobian_func', [q], [jacobian])

    # --- 5. Inverse Kinematics (Geometric Decoupling) ---
    pose_EE = ca.SX.sym('pose_EE', 5)
    
    p_EE = pose_EE[0:3]  # end-effector position
    roll = pose_EE[3]
    pitch = pose_EE[4]
    
    IK = ca.SX.zeros(6)
    
    # --- Step 1: Wrist Orientation ---
    IK[4] = pitch
    IK[5] = -roll

    # --- Step 2: Compute Wrist Position ---
    # Build transformation from wrist frame to EE frame with IK[4] and IK[5]
    H_wrist = ca.SX.eye(4)
    H_full = ca.SX.eye(4)
    
    joint_id = 4  # Start from joint 5 (index 4)
    for i in range(len(dh_params)):
        d, theta_offset, a, alpha = dh_params[i][0], dh_params[i][1], dh_params[i][2], dh_params[i][3]
        theta = theta_offset
        
        if joint_id < 6 and i == joint_indices[joint_id]:
            theta = IK[joint_id] + theta_offset
            joint_id += 1
        
        H_i = get_dh_matrix(d, theta, a, alpha)
        
        if i >= 8:  # Start of wrist/tool frame (from P4 onwards)
            H_wrist = H_wrist @ H_i
        H_full = H_full @ H_i
    
    # Transform EE position to base frame
    p_1_to_EE_baseFrame = world_to_base @ ca.vertcat(p_EE, 1)
    yaw_raw = ca.atan2(p_1_to_EE_baseFrame[1], p_1_to_EE_baseFrame[0])
    
    # Calculate lateral offset from tooltip
    p_1_to_tooltip_baseFrame = world_to_base @ ca.vertcat(H_full[0:3, 3], 1)
    lateral_offset = p_1_to_tooltip_baseFrame[1]
    
    # Calculate current position in base frame
    p_1_to_EE_current = world_to_base @ ca.vertcat(p_EE, 1)
    real_radius = ca.norm_2(p_1_to_EE_current[0:2])
    
    # Calculate angular offset
    ratio = lateral_offset / (real_radius + 1e-6)
    ratio_clamped = ca.fmin(1, ca.fmax(-1, ratio))
    tool_yaw_offset = ca.asin(ratio_clamped)
    
    yaw_actual = yaw_raw - tool_yaw_offset
    
    # Rotation matrices
    def Rx(th):
        return ca.vertcat(
            ca.horzcat(1, 0, 0),
            ca.horzcat(0, ca.cos(th), -ca.sin(th)),
            ca.horzcat(0, ca.sin(th), ca.cos(th))
        )
    
    def Ry(th):
        return ca.vertcat(
            ca.horzcat(ca.cos(th), 0, ca.sin(th)),
            ca.horzcat(0, 1, 0),
            ca.horzcat(-ca.sin(th), 0, ca.cos(th))
        )
    
    def Rz(th):
        return ca.vertcat(
            ca.horzcat(ca.cos(th), -ca.sin(th), 0),
            ca.horzcat(ca.sin(th), ca.cos(th), 0),
            ca.horzcat(0, 0, 1)
        )
    
    R_EE = Rz(yaw_actual) @ Ry(pitch) @ Rx(roll)
    
    # Wrist position in EE frame
    EE_frame_wrist_position = -(H_wrist[0:3, 0:3].T) @ H_wrist[0:3, 3]
    world_frame_wrist_position = p_EE + R_EE @ EE_frame_wrist_position
    
    # Transform wrist position to base frame
    p_1_to_wrist_baseFrame = world_to_base @ ca.vertcat(world_frame_wrist_position, 1)
    IK[0] = ca.atan2(p_1_to_wrist_baseFrame[1], p_1_to_wrist_baseFrame[0])
    
    # --- Step 3: Extract Rest Positions (q=0) ---
    p2_0 = init_joint_positions[1]
    p3_0 = init_joint_positions[2]
    p4_0 = init_joint_positions[3]
    
    vec_Upper_0 = p3_0 - p2_0
    vec_Fore_0 = p4_0 - p3_0
    
    # Link lengths
    lambda_1 = ca.norm_2(vec_Upper_0)
    lambda_2 = ca.norm_2(vec_Fore_0)
    
    # Reference elbow angle at rest
    dist_reach_0 = ca.norm_2(p4_0 - p2_0)
    cos_delta_0 = (lambda_1**2 + lambda_2**2 - dist_reach_0**2) / (2 * lambda_1 * lambda_2)
    cos_delta_0 = ca.fmin(1, ca.fmax(-1, cos_delta_0))
    delta_0 = ca.acos(cos_delta_0)
    
    # Calculate Beta (shoulder offset angle at rest)
    beta = ca.acos((vec_Upper_0.T @ frame_2_x_axis) / lambda_1)
    
    # --- Step 4: Solve for q2 and q3 ---
    # Vector from shoulder to target wrist
    p_2_to_wrist_worldFrame = world_frame_wrist_position - joint_2_pos_func(IK[0])
    lambda_3 = ca.norm_2(p_2_to_wrist_worldFrame)
    
    # Required elbow angle
    cos_ratio = (lambda_1**2 + lambda_2**2 - lambda_3**2) / (2 * lambda_1 * lambda_2)
    cos_ratio_clamped = ca.fmin(1, ca.fmax(-1, cos_ratio))
    delta_now = ca.acos(cos_ratio_clamped)
    
    # Solve Joint 3
    IK[2] = delta_0 - delta_now
    
    # Calculate shoulder triangle geometry at rest
    vec_Shoulder_Wrist_0 = p4_0 - p2_0
    D_0 = ca.norm_2(vec_Shoulder_Wrist_0[0:2])
    psi_0 = ca.atan2(vec_Shoulder_Wrist_0[2], D_0)
    
    # Internal triangle angle at shoulder at rest
    cos_phi_0 = (lambda_1**2 + dist_reach_0**2 - lambda_2**2) / (2 * lambda_1 * dist_reach_0)
    cos_phi_0 = ca.fmin(1, ca.fmax(-1, cos_phi_0))
    phi_0 = ca.acos(cos_phi_0)
    
    # Zero offset for Joint 2
    q2_offset = phi_0 + psi_0
    
    # Calculate shoulder triangle geometry for target
    p_2_to_wrist = world_frame_wrist_position - joint_2_pos_func(IK[0])
    D_target = ca.norm_2(p_2_to_wrist[0:2])
    psi_target = ca.atan2(p_2_to_wrist[2], D_target)
    
    # Internal triangle angle at shoulder for target
    phi_target = ca.asin(lambda_2 * ca.sin(delta_now) / lambda_3)
    
    # Solve Joint 2
    IK[1] = q2_offset - (phi_target + psi_target)
    
    # --- Step 5: Solve for q4 (wrist alignment) ---
    R_P3_to_world = world_to_wrist_func(IK[0:3])
    yaw_offset = ca.pi/2
    R_z = ca.vertcat(
        ca.horzcat(ca.cos(yaw_offset), ca.sin(yaw_offset), 0),
        ca.horzcat(-ca.sin(yaw_offset), ca.cos(yaw_offset), 0),
        ca.horzcat(0, 0, 1)
    )
    R_hat = R_P3_to_world @ R_z
    IK[3] = ca.atan2(-R_hat[2,0], R_hat[2,1])

    ik_func = ca.Function('IK_func', [pose_EE], [IK])

    # --- 6. Save Functions ---
    try:
        share_dir = get_package_share_directory('poker_control')
        save_dir = os.path.join(share_dir, 'models')
    except:
        save_dir = os.path.join(os.getcwd(), 'models')

    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
        
    print(f"Saving .casadi files to: {save_dir}")
    
    fk_func.save(os.path.join(save_dir, 'forward_kinematics.casadi'))
    jacobian_func.save(os.path.join(save_dir, 'jacobian.casadi'))
    ik_func.save(os.path.join(save_dir, 'inverse_kinematics.casadi'))
    
    print("Done.")

if __name__ == '__main__':
    main()