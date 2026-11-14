%% setup 4.1: steady-state Kalman filter for roll and pitch, no bias
% x = [phi; theta] (rad), u = [p_m; q_m] gyro (rad/s), y = [phi_m; theta_m] inclinometer (rad)
% used by ARDroneReplay_V2 and ARDroneHoverSim_4_1 (kalman_filter block)

% state space matrices
A = zeros(2);
B = eye(2);
C = eye(2);

% uncertainty
sensor_cov;

R = diag([R_p, R_q]);         % process noise (gyro)
Q = diag([Q_phi, Q_theta]);   % measurement noise (inclinometer)

% gain
sys = ss(A, [B eye(2)], C, 0);
[~, KALMAN_GAIN, ~] = kalman(sys, R, Q);   % kalman(sys, Qn, Rn): Qn = process = R, Rn = measurement = Q

% constant
sampleTime = 1/200;
