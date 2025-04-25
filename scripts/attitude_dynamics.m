function dq = attitude_dynamics(t, state, J, Kp, Kd, n)
    % Attitude dynamics for 3U CubeSat
    q = state(1:4); % Quaternion
    w = state(5:7); % Angular velocity
    q_d = [1; 0; 0; 0]; % Desired quaternion (nadir)
    q_e = quatmultiply(quatconj(q_d'), q')';
    tau = -Kp * q_e(2:4) - Kd * w; % PID control
    q_dot = 0.5 * quatmultiply(q', [0; w]')';
    w_dot = J \ (tau - cross(w, J * w));
    dq = [q_dot; w_dot];
end
