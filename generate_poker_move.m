function [t_vec, q_ref, q_vel] = generate_poker_move(start_pos, end_pos, duration, Ts)
    % GENERATE_POKER_MOVE
    % Generates a smooth Quintic polynomial trajectory using regression.
    % 
    % Inputs:
    %   start_pos - Joint angle at start (rad)
    %   end_pos   - Joint angle at end (rad)
    %   duration  - Total time for the move (s)
    %   Ts        - Sampling period (s)
    %
    % Outputs:
    %   t_vec - Time vector (not needed for actual robot)
    %   q_ref - Reference position vector
    %   q_vel  - Feedforward control

    %% Define Boundary Constraints (Constants)
    t0 = 0;
    tf = duration;
    
    v0 = 0;   vf = 0;   % Velocity
    a0 = 0;   af = 0;   % Acceleration
    
    %% Set up the Linear System (Ma = b)
    % We are solving for coefficients a = [a0 a1 a2 a3 a4 a5]' 
    % for the polynomial: q(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    
    % The Matrix M represents the equations for Pos, Vel, Acc at t0 and tf
    M = [
            1, t0, t0^2,   t0^3,    t0^4,    t0^5;
            0, 1,  2*t0, 3*t0^2,  4*t0^3,  5*t0^4;
            0, 0,     2, 6*t0,   12*t0^2, 20*t0^3;
            1, tf, tf^2,   tf^3,    tf^4,    tf^5;
            0, 1,  2*tf, 3*tf^2,  4*tf^3,  5*tf^4;
            0, 0,     2, 6*tf,   12*tf^2, 20*tf^3
        ];

    % The Vector b contains the boundary values
    b = [start_pos; v0; a0; end_pos; vf; af];
    
    % Solve for coefficients
    coeffs = M \ b;
    
    %% Generate Trajectory Vectors
    % Create time vector
    t_vec = 0:Ts:duration;
    
    % Evaluate Polynomial
    % q = a0 + a1*t + a2*t^2 ...
    % We can use polyval, but note MATLAB's polyval expects order [an ... a0]
    % Our coeffs are [a0 ... an], so we flip them.
    q_ref = polyval(flip(coeffs), t_vec);
    
    %% Calculate Discrete Feedforward
    q_next = [q_ref(2:end), q_ref(end)]; 
    q_vel = (q_next - q_ref) / Ts; % q_vel[k] = (q[k+1] - q[k]) / Ts
end