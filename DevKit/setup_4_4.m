%% setup file for LAB2 — 4.4 (Kalman com bias do rate gyro)
%  Estado:  x = [phi; theta; b_p; b_q]       (rad, rad, rad/s, rad/s)
%  Entrada: u = [p_m; q_m]                    (gyro, rad/s)
%  Saída:   y = [phi_m; theta_m]              (inclinómetro, rad)
%
%  phi_dot   = p_m - b_p + w1          b_p_dot = w3
%  theta_dot = q_m - b_q + w2          b_q_dot = w4
%  y         = [phi; theta] + n

% state space matrices
A = [0 0 -1  0;
     0 0  0 -1;
     0 0  0  0;
     0 0  0  0];
B = [eye(2); zeros(2)];
C = [eye(2) zeros(2)];

% observabilidade (precisa ser 4)
fprintf('rank(obsv(A,C)) = %d\n', rank(obsv(A, C)));

% uncertainty (exp. C em hover, ver sensor_cov.m / sec3_stats.m)
sensor_cov;
kb   = 1e-2;         % R_b = kb*R_gyro: afinação, tau_bias ~ 1/sqrt(kb) = 10 s (ver plot_sim.m)
R_bp = kb*R_p;       % (rad/s^2)^2  passeio aleatório do bias
R_bq = kb*R_q;

R = diag([R_p, R_q, R_bp, R_bq]);   % ruido de processo (gyro + bias)
Q = diag([Q_phi, Q_theta]);         % ruido de medida (inclinometro)

% ruído de processo entra em todos os 4 estados -> G = eye(4)
sys = ss(A, [B eye(4)], C, 0);
[~, KALMAN_GAIN, ~] = kalman(sys, R, Q);   % R=processo, Q=medida;   % 4×2

% constant
sampleTime = 1/200;
