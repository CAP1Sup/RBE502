function [F, M, trpy, drpy] = pid_controller(qd, t, qn, params)
    % CONTROLLER quadrotor controller
    % The current states are:
    % qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
    % The desired states are:
    % qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
    % Using these current and desired states, you have to compute the desired controls

    % =================== Your code goes here ===================

    persistent desired_angles;
    persistent icnt;

    % Increment the counter
    icnt = icnt + 1;

    % Initialize the desired angles
    if isempty(desired_angles)
        desired_angles = zeros(0, 3);
        icnt = 5; % Force the first iteration to update the attitude
    end

    % Coefficients for the position PD controller
    posP = 15;
    posD = 5;
    % In form: [pX, dX; pY, dY; pZ, dZ]
    kp = [posP, posD;
          posP, posD;
          100, 20];

    % Coefficients for the attitude PD controller
    % In form: [pRoll, dRoll; pPitch, dPitch; pYaw, dYaw]
    attP = 3000;
    attD = attP / 10;
    ka = [attP, attD;
          attP, attD;
          attP, attD];

    % Readability
    phi = qd{qn}.euler(1); % Roll
    theta = qd{qn}.euler(2); % Pitch
    %psi = qd{qn}.euler(3); Yaw, not used

    kpP = kp(:, 1);
    kpD = kp(:, 2);
    kaP = ka(:, 1);
    kaD = ka(:, 2);

    % Desired accelerations (general form)
    acc_des = qd{qn}.acc_des + kpD .* (qd{qn}.vel_des - qd{qn}.vel) + kpP .* (qd{qn}.pos_des - qd{qn}.pos);

    % Position Controller
    % Uses a 3D trajectory controller to determine the desired angles and angular velocities
    % Uses a PD controller to determine the moments
    % Determine if we need to update the attitude
    if (icnt == 5)
        icnt = 0;

        % Desired angles
        phi_des = 1 / params.grav * (acc_des(1) * sin(qd{qn}.yaw_des) - acc_des(2) * cos(qd{qn}.yaw_des));
        theta_des = 1 / params.grav * (acc_des(1) * cos(qd{qn}.yaw_des) + acc_des(2) * sin(qd{qn}.yaw_des));
        psi_des = qd{qn}.yaw_des;

        % Bound the angles
        max_angle = 60 * pi / 180;
        phi_des = max(min(phi_des, max_angle), -max_angle);
        theta_des = max(min(theta_des, max_angle), -max_angle);

        % Update the desired angles
        desired_angles = [phi_des; theta_des; psi_des];
    end

    % Desired angular velocities
    p_des = 0;
    q_des = 0;
    r_des = qd{qn}.yawdot_des;

    % Assemble angular velocities into a useful form
    desired_angular_vels = [p_des; q_des; r_des];

    % Attitude Control
    M = params.I * (kaP .* (desired_angles - qd{qn}.euler) + kaD .* (desired_angular_vels - qd{qn}.omega));

    % Thrust Control
    % Vector compensation for the angle of the quadrotor
    F = params.mass * params.grav + params.mass * acc_des(3) * cos(phi) * cos(theta);

    % Output trpy and drpy as in hardware
    % Desired thrust, roll, pitch, and yaw
    trpy = [0, 0, 0, 0];

    % Derivatives of desired thrust, roll, pitch, and yaw
    drpy = [0, 0, 0, 0];
end
