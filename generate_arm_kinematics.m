clc;
clear;
close all

import casadi.*

% number of joint of the n-link arm
n_joints = 6;

% Symbolic variables for the robot's state
q = SX.sym('q', [n_joints, 1]); % Joint positions

%% Robot Link Parameters
% (x_0, y_0) is the position of the robot's base in the world frame.
x_0 = 0.5;
y_0 = 0.2;

l_1 = 0.07;
l_2 = 0.07;
l_3 = 0.03;
l_4 = 0.115;
l_5 = 0.03;
l_6 = 0.137;
l_7 = 0.0045;
l_8 = 0.06;
l_9 = 0.03;
l_10 = 0.02;
l_11 = 0.055;
l_12 = 0.055;

%% Robot Forward Kinematics (Denavit-Hartenberg Parameters)
% Defined for the ARM project's modified Hugging Face robot
% Standard DH parameters [d, theta_offset, a, alpha]

dh_table = [  0,           0,      x_0,     0;     % WtoP1
            l_1,        pi/2,      y_0,     0;     % P1toB
              0,           0,        0,     0;     % Bto1
            l_2,           0,      l_3, -pi/2;     % 1toP2
              0,           0,      l_4,     0;     % P2to2
              0,       -pi/2,      l_5,     0;     % 2toP3
              0,        pi/2,      l_6,     0;     % P3to3
              0,       -pi/2,      l_7,     0;     % 3toP4
              0,        pi/2,      l_8,     0;     % P4to4
              0,       -pi/2,        0, -pi/2;     % 4toP5
              0,           0,        0,     0;     % P5to5
            l_9,           0,     l_10,  pi/2;     % 5toP6
              0,        pi/2,     l_11,     0;     % P6to6
              0,        pi/2,     l_12,     0;     % 6toalign1
              0,       -pi/2,        0,  pi/2;     % align1toalign2
              0,       -pi/2,        0,     0;     % align2toEE
           ];

% Initial Homogenous Transformation Matrix
H = eye(4);
H_init = eye(4);

joint_ID = 1;
joint_info = [3, 5, 7, 9, 11, 13];
init_joint_positions = cell(n_joints, 1);
joint_positions = cell(n_joints, 1);
joint_rotation_axes = cell(n_joints, 1);

%% Recursive Homogenous Transformation
for i = 1:length(dh_table)
    % Extract DH parameters for the current link
    d = dh_table(i, 1);
    theta = dh_table(i, 2);
    theta_offset = theta;
    a = dh_table(i, 3);
    alpha = dh_table(i, 4);

    % Homogeneous transformation matrix using the standard DH convention
    if joint_ID <= n_joints
        if i == joint_info(joint_ID)
            theta = q(joint_ID) + theta;
            init_joint_positions{joint_ID}  = H_init(1:3, 4);
            joint_positions{joint_ID} = H(1:3, 4);
            joint_rotation_axes{joint_ID} = H(1:3, 3);
            if joint_ID == 1
                base_to_world = H;
                world_to_base = [H(1:3, 1:3)', -H(1:3, 1:3)'*H(1:3, 4);
                                  zeros(1, 3),                       1];
            end
            if joint_ID == 2
                frame_2_x_axis = H_init(1:3, 1);
            end
            if joint_ID == 3
                frame_3_x_axis = H_init(1:3, 1);
            end
            if joint_ID == 4
                joint_3_R = H(1:3, 1:3);
            end
            joint_ID = joint_ID + 1;
        end
    end
    H_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                    0,             sin(alpha),             cos(alpha),            d;
                    0,                      0,                      0,            1;
          ];
    H_init_i = [cos(theta_offset), -sin(theta_offset)*cos(alpha),  sin(theta_offset)*sin(alpha),...
                    a*cos(theta_offset);
                sin(theta_offset),  cos(theta_offset)*cos(alpha), -cos(theta_offset)*sin(alpha),...
                    a*sin(theta_offset);
                                0,             sin(alpha),             cos(alpha),            d;
                                0,                      0,                      0,            1;
               ];
    H = H*H_i;
    H_init = H_init*H_init_i;
end

% Actual position of joint 2
joint_2_pos_var = joint_positions{2};
joint_2_pos_func = Function('joint_2_pos_func', {q(1)}, {joint_2_pos_var});

% Rotation matrix from world frame to wrist frame
world_to_wrist_func = Function('world_to_wrist_func', {q(1:3)}, {joint_3_R});

% End Effector's position in frame 0
world_frame_EE_position = H(1:3, 4);

% End Effector's orientation using Euler angles in frame 0
R_ee = H(1:3, 1:3);
roll  = atan2(R_ee(3,2), R_ee(3,3));
pitch = asin(-R_ee(3,1));
yaw   = atan2(R_ee(2,1), R_ee(1,1));
world_frame_EE_orientation = [roll; pitch; yaw];

cartesian_state = [world_frame_EE_position; world_frame_EE_orientation]; 
FK_func = Function('FK_func', {q}, {cartesian_state});
% Example usage: 
% cart_state = full(FK_func([0.3; -0.2; 0; 0.2; 0.2; 0.15]));
% 0.3; 0.2; 0; 0.1; 0.2
% NB: make sure q4 is defined such that it is parallel to the xy-plane,
% else you will get errors in the inverse kinematics.

%% Inverse Kinematics
pose_EE = SX.sym('pose_EE', [5, 1]);

p_EE = pose_EE(1:3); % end-effector position in world frame

roll = pose_EE(4);
R_x = [ 1,          0,         0;
        0,  cos(roll), -sin(roll);
        0,  sin(roll),  cos(roll)];

pitch = pose_EE(5);
R_y = [ cos(pitch), 0, sin(pitch);
                 0, 1,          0;
       -sin(pitch), 0, cos(pitch)];

IK = SX.zeros(n_joints, 1);

IK(5) = pitch;
IK(6) = -roll;

% Transformation from wrist frame to end-effector frame
joint_ID = 5;
H_wrist = eye(4); % wrist frame
H_full = eye(4); % Ghost robot with tool activated
for i = 1:length(dh_table)
    % Extract DH parameters for the current link
    d = dh_table(i, 1);
    theta = dh_table(i, 2);
    a = dh_table(i, 3);
    alpha = dh_table(i, 4);

    if joint_ID <= n_joints
        if i == joint_info(joint_ID)
            theta = IK(joint_ID) + theta;
            joint_ID = joint_ID + 1;
        end
    end

    % Homogeneous transformation matrix using the standard DH convention
    H_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
                    0,             sin(alpha),             cos(alpha),            d;
                    0,                      0,                      0,            1;
          ];
    if i >= 9 % Start of tool frame
        H_wrist = H_wrist*H_i;
    end
    H_full = H_full*H_i;
end

p_1_to_EE_baseFrame = world_to_base*[p_EE; 1];
yaw_raw = atan2(p_1_to_EE_baseFrame(2), p_1_to_EE_baseFrame(1));

p_1_to_tooltip_baseFrame = world_to_base * [H_full(1:3, 4); 1];
lateral_offset = p_1_to_tooltip_baseFrame(2);

p_1_to_EE_current = world_to_base * [p_EE; 1];
% Horizontal distance from base to EE
real_radius = norm(p_1_to_EE_current(1:2));

% Calculate the angular offset
ratio = lateral_offset / (real_radius + 1e-6); % Add epsilon to avoid divide-by-zero
ratio_clamped = fmin(1, fmax(-1, ratio));      % Clamp between -1 and 1
tool_yaw_offset = asin(ratio_clamped);

yaw_actual = yaw_raw - tool_yaw_offset;
R_z = [ cos(yaw_actual), -sin(yaw_actual), 0;
        sin(yaw_actual),  cos(yaw_actual), 0;
                      0,          0, 1];

% Align tool frame with actual arm
R_EE = R_z*R_y*R_x; 

EE_frame_wrist_position = -(H_wrist(1:3, 1:3)')*H_wrist(1:3, 4);
world_frame_wrist_position = p_EE + R_EE*EE_frame_wrist_position;

p_1_to_wrist_baseFrame = world_to_base*[world_frame_wrist_position; 1];
IK(1) = atan2(p_1_to_wrist_baseFrame(2), p_1_to_wrist_baseFrame(1));

% Extract vectors for the arm at rest (q=0) using initial FK positions
% These are numerical values calculated during the setup loop
p2_0 = init_joint_positions{2};
p3_0 = init_joint_positions{3};
p4_0 = init_joint_positions{4};

vec_Upper_0 = p3_0 - p2_0; % Vector P2->P3 at rest
vec_Fore_0  = p4_0 - p3_0; % Vector P3->P4 at rest

% Calculate Link Lengths (Constants)
lambda_1 = norm(vec_Upper_0);
lambda_2 = norm(vec_Fore_0);

% Calculate the "Reference" Elbow Angle (delta_0) using Law of Cosines
% We calculate the internal angle of the elbow when q=0
dist_reach_0 = norm(p4_0 - p2_0); % Distance P2->P4 at rest

% Law of Cosines: c^2 = a^2 + b^2 - 2ab*cos(angle)
cos_delta_0 = (lambda_1^2 + lambda_2^2 - dist_reach_0^2) / (2 * lambda_1 * lambda_2);
cos_delta_0 = min(1, max(-1, cos_delta_0)); 
delta_0 = acos(cos_delta_0);

% Calculate Beta (Shoulder offset angle at rest)
beta = acos((vec_Upper_0' * frame_2_x_axis) / lambda_1);

% Vector from Shoulder (P2) to Target Wrist
p_2_to_wrist_worldFrame = world_frame_wrist_position - joint_2_pos_func(IK(1));
lambda_3 = norm(p_2_to_wrist_worldFrame);

% Calculate Required Elbow Angle (delta_now)
cos_ratio = (lambda_1^2 + lambda_2^2 - lambda_3^2) / (2 * lambda_1 * lambda_2);
cos_ratio_clamped = fmin(1, fmax(-1, cos_ratio));
delta_now = acos(cos_ratio_clamped);

% Solve Joint 3: The difference between the rest angle and required angle
IK(3) = delta_0 - delta_now;

% Calculate the Shoulder Triangle Geometry at REST (q=0)
vec_Shoulder_Wrist_0 = p4_0 - p2_0;

% Elevation of the wrist at rest (psi_0)
D_0 = norm(vec_Shoulder_Wrist_0(1:2));
psi_0 = atan2(vec_Shoulder_Wrist_0(3), D_0);

% Internal triangle angle at shoulder at rest (phi_0)
cos_phi_0 = (lambda_1^2 + dist_reach_0^2 - lambda_2^2) / (2 * lambda_1 * dist_reach_0);
cos_phi_0 = min(1, max(-1, cos_phi_0));
phi_0 = acos(cos_phi_0);

% Define the "Zero Offset" for Joint 2
% This represents the total global angle of the upper arm when q2 = 0
q2_offset = phi_0 + psi_0; 

% Calculate the Shoulder Triangle Geometry for TARGET
p_2_to_wrist = world_frame_wrist_position - joint_2_pos_func(IK(1));
D_target = norm(p_2_to_wrist(1:2));
psi_target = atan2(p_2_to_wrist(3), D_target);

% Internal triangle angle at shoulder for target (phi_target)
phi_target = asin(lambda_2 * sin(delta_now) / lambda_3);

% Solve Joint 2
IK(2) = q2_offset - (phi_target + psi_target);

R_P3_to_world = world_to_wrist_func(IK(1:3));
yaw_offset = pi/2;
R_z = [ cos(yaw_offset), sin(yaw_offset), 0;
       -sin(yaw_offset), cos(yaw_offset), 0;
                      0,        0, 1];
R_hat = R_P3_to_world*R_z;
% Define joint 4 such that it is always parallel to the x-y plane
IK(4) = atan2(-R_hat(3,1), R_hat(3,2));

IK_func = Function('IK_func', {pose_EE}, {IK});
% Example usage: 
% joint_angles = full(IK_func(cart_state(1:5)));
% NB: must run the forward kinematics function first to get cart_state

%% Robot Velocity (Jacobian)
axes = SX.sym('axes', [3, 1]);
skew_matrix = [       0, -axes(3),  axes(2);
                axes(3),        0, -axes(1);
               -axes(2),  axes(1),        0
              ];
skew_matrix_func = Function('skew_matrix_func', {axes}, {skew_matrix});

J_p = SX.zeros(3, n_joints);
J_o = SX.zeros(3, n_joints);

for i = 1:n_joints
    J_p(:, i) = skew_matrix_func(joint_rotation_axes{i})*(world_frame_EE_position...
                                                            - joint_positions{i});
    J_o(:, i) = joint_rotation_axes{i};
end
jacobian = [J_p; J_o];
jacobian_func = Function('jacobian_func', {q}, {jacobian});
% Example usage: 
% jacobian_mat = full(jacobian_func(zeros(n_joints, 1)));
% NB: Be careful of kinematic singularities

%% Export CasADi functions
FK_func.save('forward_kinematics.casadi');
IK_func.save('inverse_kinematics.casadi');
jacobian_func.save('jacobian.casadi');