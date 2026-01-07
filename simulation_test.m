clear; clc; close all;
import casadi.*

%% Robot & Simulation Parameters
n_joints = 6;

% Timing
Ts = 0.01;           % Controller runs at 100 Hz
Duration = 1.0;       % 1 second move

% Kinematics Functions
FK_func = Function.load('forward_kinematics.casadi'); % forward kinematics
IK_func = Function.load('inverse_kinematics.casadi'); % inverse kinematics

%% Controller Design (Discrete LQR)
% System: q_dot = u  => Discrete: q_{k+1} = q_k + Ts*u_k
A_d = eye(n_joints);
B_d = Ts*A_d;

Q = 100*eye(n_joints);   % Penalty on state error    
R = 1*eye(n_joints);   % Penalty on control effort

% Solve DARE (Discrete Algebraic Riccati Equation)
P = (Q ./ 2) + sqrt((Q.^2 ./ 4) + (Q .* R) ./ (Ts^2));
K = (Ts * P) / (R + Ts^2 * P);

%% Trajectory Generation (Quintic Polynomial in Joint Space)
% Define Start and End Configurations
q_start = zeros(6, 1);
cart_state = full(FK_func([0.3; -0.2; 0; 0.2; 0.2; 0.15]));
q_end = full(IK_func(cart_state(1:5)));
  
% Pre-allocate
t_vec = 0:Ts:Duration;
steps = length(t_vec);
q_ref = zeros(n_joints, steps);
q_vel  = zeros(n_joints, steps);

% Loop through each joint and generate its specific trajectory
for j = 1:n_joints
    [~, q_j, u_j] = generate_poker_move(q_start(j), q_end(j), Duration, Ts);
    q_ref(j, :) = q_j;
    q_vel(j, :) = u_j;
end

%% Simulation
% Initial State
q_current = q_start; 

% History Storage
q_hist = zeros(n_joints, steps);
t_hist = zeros(1, steps);
u_hist = zeros(n_joints, steps);

% Simulation Loop
for k = 1:steps
    % --- DISCRETE CONTROLLER UPDATE (ZOH) ---
    % Calculate Error
    e_k = q_current - q_ref(:, k);
    
    % Control Law: u = q_vel - K*e
    u_k = q_vel(:, k) - K * e_k;
    
    % Store Data
    q_hist(:, k) = q_current;
    u_hist(:, k) = u_k;
    t_hist(k)    = t_vec(k);
    
    % --- DISCRETE PLANT SIMULATION ---
    % Simulate from t_k to t_{k+1} assuming u_k is constant (ZOH)
    if k < steps
        % Update state for next iteration
        q_current = q_ref(:, k+1) + (A_d - B_d*K)*e_k;
    end
end
% NB: The actual STS3215 motors do NOT have a velocity control mode.
% Thus instead of passing u_k to the motors, we will actually be passing:
% q_{cmd} = q_{d,k+1} + (A_d - B_d*K)*e_k to the motors.
% We also set the max velocity to be equal to:
% q_{lim} = min(q_{max}, max(1.5*abs(u_k), q_{min})) at any time step k.

%% Forward Kinematics & Plotting
% This creates a new function that accepts n_jointsxN and returns 6xN
N_points = size(q_hist, n_joints);
FK_map = FK_func.map(N_points);
task_space = full(FK_map(q_hist));

for j = 1:n_joints
    figure('Color', 'w', 'Name', sprintf('Joint %d Analysis', j));
    
    % Subplot 1: Position Tracking
    subplot(2,1,1);
    hold on;
    plot(t_hist, q_hist(j,:), 'b', 'LineWidth', 1.5); 
    plot(t_hist, q_ref(j,:), 'r--', 'LineWidth', 1.5);
    hold off;
    title(sprintf('Joint %d Position Tracking', j)); 
    legend('Actual', 'Reference', 'Location', 'best'); 
    ylabel('Angle (rad)');
    grid on;
    
    % Subplot 2: Robot velocity
    subplot(2,1,2);
    plot(t_hist, u_hist(j,:), 'k', 'LineWidth', 1.0);
    title(sprintf('Joint %d Velocity', j)); 
    xlabel('Time (s)'); 
    ylabel('Joint Velocity (rad/s)'); 
    grid on;
end

% --- Task Space Trajectory ---
figure('Color', 'w', 'Name', 'Poker Dealer Animation');
grid on; axis equal; hold on;
view(3); % Set 3D view
view(-90, 30);

% Setup Static Elements
% Plot the full path as a faint trace so we see the "road"
plot3(task_space(1,:), task_space(2,:), task_space(3,:), ...
      'Color', [0.8 0.8 0.8], 'LineWidth', 1.5, 'LineStyle', '--');

% Set fixed axis limits so the camera doesn't jump around
margin = 0.1;
xlim([min(task_space(1,:))-margin, max(task_space(1,:))+margin]);
ylim([min(task_space(2,:))-margin, max(task_space(2,:))+margin]);
zlim([min(task_space(3,:))-margin, max(task_space(3,:))+margin]);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End-Effector Trajectory (Green Arrow = Local Y-Axis)');

% Initialize the Moving Objects
% The "Gripper" point (Red Dot)
h_point = plot3(0, 0, 0, 'ro', 'MarkerSize', 6, 'MarkerFaceColor', 'r');

% The "Direction" Arrow (Green Arrow representing Local Y-axis)
% quiver3(x, y, z, u, v, w) -> starts at xyz, points in uvw direction
scale = 0.1; % Length of the arrow (10cm)
h_arrow = quiver3(0, 0, 0, 0, 0, 0, ...
    'Color', 'g', 'LineWidth', 2.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off');

% 3. Animation Loop
fprintf('Starting Animation...\n');
for k = 1:5:length(t_vec) % Skip frames for speed (optional)
    
    % --- Extract State ---
    pos = task_space(1:3, k); % Current [x; y; z]
    eul = task_space(4:6, k); % Current [roll; pitch; yaw]
    
    % --- Calculate Orientation Vector ---
    % We need to rotate the Local Y-vector [0; 1; 0] into World Frame.
    % Use a helper to get Rotation Matrix R from Euler (ZYX convention standard)
    R = eul2rotm_custom(eul); 
    
    % Local Y-axis is column 2 of the Rotation Matrix
    dir_vec = R(:, 2) * scale; 
    
    % --- Update Graphics Objects ---
    % Update Point Position
    set(h_point, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    
    % Update Arrow Position (Start Point)
    set(h_arrow, 'XData', pos(1), 'YData', pos(2), 'ZData', pos(3));
    
    % Update Arrow Direction (Vector components)
    set(h_arrow, 'UData', dir_vec(1), 'VData', dir_vec(2), 'WData', dir_vec(3));
    
    % Update Title
    title(sprintf('Time: %.2fs | Pose: [%.2f %.2f %.2f]', t_vec(k), pos(1), pos(2), pos(3)));
    
    % Force render
    drawnow limitrate; 
    
    pause(0.05);
end
fprintf('Animation Complete.\n');

% --- Helper Function: Euler ZYX to Rotation Matrix ---
function R = eul2rotm_custom(eul)
    % Assumes eul = [roll; pitch; yaw] (rad)
    % R = Rz(y) * Ry(p) * Rx(r)
    phi = eul(1); theta = eul(2); psi = eul(3);
    
    Rx = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    Ry = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    Rz = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    
    R = Rz * Ry * Rx;
end