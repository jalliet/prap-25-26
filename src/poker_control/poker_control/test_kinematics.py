#!/usr/bin/env python3
import sys
import os
import numpy as np
import casadi as cs
import time

def get_model_path(model_name):
    """
    Attempts to locate the .casadi model file.
    Priority 1: ROS 2 Package Share (if installed)
    Priority 2: Local 'models' directory (standalone execution)
    """
    # 1. Try ROS 2 Share path
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory('poker_control')
        path = os.path.join(share_dir, 'models', model_name)
        if os.path.exists(path):
            return path
    except (ImportError, LookupError):
        pass

    # 2. Try local path (relative to where script is run)
    local_path = os.path.join(os.getcwd(), 'models', model_name)
    if os.path.exists(local_path):
        return local_path
    
    # 3. Try standard install path structure if running from src
    # ../../../install/poker_control/share/poker_control/models
    install_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 
                                                '../../../../install/poker_control/share/poker_control/models', 
                                                model_name))
    if os.path.exists(install_path):
        return install_path

    return None

def main():
    print("========================================")
    print("   Poker Arm Kinematics Validator")
    print("========================================")

    # 1. Load Models
    fk_path = get_model_path('forward_kinematics.casadi')
    ik_path = get_model_path('inverse_kinematics.casadi')
    jac_path = get_model_path('jacobian.casadi')

    if not fk_path or not ik_path:
        print(f"[ERROR] Could not find .casadi models.")
        print("Please run: ros2 run poker_control generate_kinematics")
        return

    print(f"[INFO] Loading FK: {fk_path}")
    fk_solver = cs.Function.load(fk_path)
    
    print(f"[INFO] Loading IK: {ik_path}")
    ik_solver = cs.Function.load(ik_path)

    print(f"[INFO] Loading Jacobian: {jac_path}")
    jac_solver = cs.Function.load(jac_path) if jac_path else None

    # 2. Define Test Configurations (Joint Angles in Radians)
    test_configs = [
        ("Home", np.zeros(6)),
        ("Complex Pose", np.array([0.5, -0.5, 1.0, -0.5, 0.5, 0.2]))
    ]

    # 3. Run Round-Trip Tests
    all_passed = True
    
    print("\n--- Running Round-Trip Tests (Joints -> FK -> Pose -> IK -> Verify Pose) ---")
    
    for name, q_in in test_configs:
        print(f"\nTesting: {name}")
        print(f"  Input Joints: {np.round(q_in, 3)}")

        # A. Forward Kinematics
        # Returns [x, y, z, r11, r12, r13, r21, r22, r23, r31, r32, r33] usually, 
        # or [x, y, z, roll, pitch, yaw] depending on your generator.
        # Assuming your generator outputs full pose vector.
        try:
            fk_res = fk_solver(q_in)
            fk_res_np = np.array(fk_res).flatten()
            
            # Extract Position (first 3 elements)
            pos = fk_res_np[:3]
            # Extract Orientation (Euler ZYX or Rotation Matrix depending on generator)
            # For IK input, we usually need [x, y, z, pitch, roll] or similar based on your IK solver inputs
            
            print(f"  FK Position:  {np.round(pos, 3)}")
        except Exception as e:
            print(f"  [FAIL] FK Computation Error: {e}")
            all_passed = False
            continue

        # B. Inverse Kinematics
        # We pass the FK result back into IK
        try:
            # Note: You need to know exactly what your IK solver expects as input.
            # Assuming it takes the full output of the FK solver (target pose vector)
            ik_res = ik_solver(fk_res_np[:5]) 
            q_out = np.array(ik_res).flatten()
            
            print(f"  IK Result:    {np.round(q_out, 3)}")
        except Exception as e:
            print(f"  [FAIL] IK Computation Error: {e}")
            all_passed = False
            continue

        # C. Verification (Forward Kinematics on IK result)
        # We don't compare q_in vs q_out directly because multiple joint configs can reach same pose.
        # Instead, we check if FK(q_out) matches FK(q_in).
        fk_check = np.array(fk_solver(q_out)).flatten()
        
        # Calculate Error (Euclidean distance for position)
        pos_error = np.linalg.norm(fk_res_np[:3] - fk_check[:3])
        
        # Check tolerance (e.g., 1mm)
        if pos_error < 1e-4:
            print(f"  [PASS] Position Error: {pos_error:.6f} m")
        else:
            print(f"  [FAIL] Position Error: {pos_error:.6f} m (Too High!)")
            all_passed = False

        # D. Jacobian Check (Optional)
        if jac_solver:
            J = jac_solver(q_in)
            if J.shape == (6, 6):
                print("  [PASS] Jacobian Dimension (6x6)")
            else:
                 # It might be 3x6 if position only, etc.
                print(f"  [INFO] Jacobian Dimension {J.shape}")

    print("\n========================================")
    if all_passed:
        print("SUCCESS: All kinematic tests passed.")
    else:
        print("FAILURE: Some tests failed. Check DH parameters or solver logic.")
    print("========================================")

if __name__ == '__main__':
    main()
