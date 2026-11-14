%% setup 4.4: Kalman filter with rate gyro bias, roll and pitch
% x = [phi; theta; b_p; b_q] (rad, rad/s), u = [p_m; q_m] (rad/s), y = [phi_m; theta_m] (rad)
%   phi_dot   = p_m - b_p + w1      b_p_dot = w3
%   theta_dot = q_m - b_q + w2      b_q_dot = w4
% used by ARDroneReplay_V2 and ARDroneHoverSim_4_4 (kalman_filter block)

% state space matrices
A = [0 0 -1  0;
     0 0  0 -1;
     0 0  0  0;
     0 0  0  0];
B = [eye(2); zeros(2)];
C = [eye(2) zeros(2)];

% observability (must be 4)
fprintf('rank(obsv(A,C)) = %d\n', rank(obsv(A, C)));

% uncertainty
sensor_cov;
kb   = 1e-2;         % R_b = kb*R_gyro, bias time constant ~ 1/sqrt(kb) = 10 s (see plot_sim.m)
R_bp = kb*R_p;       % (rad/s^2)^2  bias random walk
R_bq = kb*R_q;

R = diag([R_p, R_q, R_bp, R_bq]);   % process noise (gyro + bias)
Q = diag([Q_phi, Q_theta]);         % measurement noise (inclinometer)

% gain: process noise enters the 4 states (G = I)
sys = ss(A, [B eye(4)], C, 0);
[~, KALMAN_GAIN, ~] = kalman(sys, R, Q);   % 4x2

% constant
sampleTime = 1/200;
