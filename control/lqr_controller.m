function [F, M, trpy, drpy] = lqr_controller(qd, t, qn, params, trajhandle)
    % CONTROLLER quadrotor controller
    % The current states are:
    % qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
    % The desired states are:
    % qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
    % Using these current and desired states, you have to compute the desired controls

    % =================== Your code goes here ===================
    persistent K

    if isempty(K)
        disp('Calculating LQR controller');

        g = params.grav;

        % Define the state-space representation
        A = [0 0 0 1 0 0 0 0 0 0 0 0; % x
             0 0 0 0 1 0 0 0 0 0 0 0; % y
             0 0 0 0 0 1 0 0 0 0 0 0; % z
             0 0 0 0 0 0 0 g 0 0 0 0; % x_dot
             0 0 0 0 0 0 -g 0 0 0 0 0; % y_dot
             0 0 0 0 0 0 0 0 0 0 0 0; % z_dot
             0 0 0 0 0 0 0 0 0 1 0 0; % phi
             0 0 0 0 0 0 0 0 0 0 1 0; % theta
             0 0 0 0 0 0 0 0 0 0 0 1; % psi
             0 0 0 0 0 0 0 0 0 0 0 0; % phi_dot
             0 0 0 0 0 0 0 0 0 0 0 0; % theta_dot
             0 0 0 0 0 0 0 0 0 0 0 0]; % psi_dot

        % Define convenience variables
        Mx = 1 / params.I(1, 1);
        My = 1 / params.I(2, 2);
        Mz = 1 / params.I(3, 3);
        mI = 1 / params.mass; % Inverse mass

        % U1, U2, U3, U4
        B = [0 0 0 0; % x
             0 0 0 0; % y
             0 0 0 0; % z
             0 0 0 0; % x_dot
             0 0 0 0; % y_dot
             mI 0 0 0; % z_dot
             0 0 0 0; % phi
             0 0 0 0; % theta
             0 0 0 0; % psi
             0 Mx 0 0; % phi_dot
             0 0 My 0; % theta_dot
             0 0 0 Mz]; % psi_dot

        % Define the cost function
        Q = diag([100, 100, 100, 0.001, 0.001, 0.001, ...
                      0.001, 0.001, 0.001, 0.001, 0.001, 0.001]);
        R = diag([0.01, 0.01, 0.01, 0.01]);

        % Compute the LQR gain matrix
        K = lqr(A, B, Q, R);
    end

    % Current state
    x = [qd{qn}.pos; qd{qn}.vel; qd{qn}.euler; qd{qn}.omega];

    % Desired state
    x_des = [qd{qn}.pos_des; qd{qn}.vel_des; zeros(2, 1); qd{qn}.yaw_des; zeros(2, 1); qd{qn}.yawdot_des];

    % Compute the control output
    u = -K * (x - x_des);

    % Extract thrust and moments
    F = u(1) + params.mass * params.grav;
    M = u(2:4);

    % Output trpy and drpy as in hardware
    % Desired thrust, roll, pitch, and yaw
    trpy = [0, 0, 0, 0];

    % Derivatives of desired thrust, roll, pitch, and yaw
    drpy = [0, 0, 0, 0];
end
