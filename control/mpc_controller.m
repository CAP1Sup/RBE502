function [F, M, trpy, drpy] = mpc_controller(qd, t, qn, params, trajhandle)
    % CONTROLLER quadrotor controller
    % The current states are:
    % qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
    % The desired states are:
    % qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
    % Using these current and desired states, you have to compute the desired controls

    % =================== Your code goes here ===================
    persistent gd;
    persistent icnt;

    if isempty(gd)
        gd = zeros(0, 3);
        icnt = 0;
    end

    icnt = icnt + 1;

    %% Parameter Initialization

    if ~isempty(t)
        desired_state = trajhandle(t, qn);
    end

    F = 0;
    M = [0; 0; 0];

    %Output trpy and drpy as in hardware
    trpy = [0, 0, 0, 0];
    drpy = [0, 0, 0, 0];

end
