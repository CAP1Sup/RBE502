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

    % =================== Your code starts here ===================
    % yaw is psi
    % roll is phi
    % pitch is theta

    % Coefficents for the moment PD controller
    % In form: [pRoll, dRoll; pPitch, dPitch; pYaw, dYaw]
    rotP = 5;
    rotD = 7.5;
    km = [rotP, rotD;
          rotP, rotD;
          1, 2];

    % Coefficients for the position PD controller (only in z)
    kz = [5, 2];

    % Readability
    phi = qd{qn}.euler(1);
    theta = qd{qn}.euler(2);
    %psi = qd{qn}.euler(3); Not used

    kmP = km(:, 1);
    kmD = km(:, 2);
    kzP = kz(1);
    kzD = kz(2);

    % Position Controller
    % Uses a hover controller to determine the desired angles and angular velocities
    % Uses a PD controller to determine the moments
    % Determine if we need to update the attitude
    if (icnt == 5)
        icnt = 0;
        % Zero the desired velocities and accelerations
        % Hovering requires no movement or acceleration
        qd{qn}.vel_des = [0; 0; 0];
        qd{qn}.acc_des = [0; 0; 0];

        % Desired accelerations (general form)
        acc_des = qd{qn}.acc_des + kmD .* (qd{qn}.vel_des - qd{qn}.vel) + kmP .* (qd{qn}.pos_des - qd{qn}.pos);

        % Desired angles
        phi_des = 1 / params.grav * (acc_des(1) * sin(qd{qn}.yaw_des) - acc_des(2) * cos(qd{qn}.yaw_des));
        theta_des = 1 / params.grav * (acc_des(1) * cos(qd{qn}.yaw_des) + acc_des(2) * sin(qd{qn}.yaw_des));
        psi_des = qd{qn}.yaw_des;

        % Update the desired angles
        desired_angles = [phi_des; theta_des; psi_des];
    end

    % Desired angular velocities
    p_des = 0;
    q_des = 0;
    r_des = qd{qn}.yawdot_des;

    % Assemble angular velocities into a useful form
    desired_angular_vels = [p_des; q_des; r_des];

    % Calculations
    angular_vels = [cos(theta), 0, -cos(phi) * sin(theta);
                    0, 1, sin(phi);
                    sin(theta), 0, cos(phi) * cos(theta)] * qd{qn}.omega;

    % Attitute Control
    M = params.I * (kmP .* (desired_angles - qd{qn}.euler) + kmD .* (desired_angular_vels - angular_vels));

    % Thrust Control
    F = params.mass * params.grav - params.mass * (kzD * (qd{qn}.vel(3) - qd{qn}.vel_des(3)) + kzP * (qd{qn}.pos(3) - qd{qn}.pos_des(3)));

    % =================== Your code ends here ===================

    % Output trpy and drpy as in hardware
    % Desired thrust, roll, pitch, and yaw
    trpy = [0, 0, 0, 0];

    % Derivatives of desired thrust, roll, pitch, and yaw
    drpy = [0, 0, 0, 0];
end
