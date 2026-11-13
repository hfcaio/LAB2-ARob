%% setup file for LAB2 — 4.1 (Kalman sem bias)
%  Estado:  x = [phi; theta]       (rad)
%  Entrada: u = [p_m; q_m]         (gyro, rad/s)
%  Saída:   y = [phi_m; theta_m]   (inclinómetro, rad)

% state space matrices
A = zeros(2);
B = eye(2);
C = eye(2);

% uncertainty (exp. C em hover, ver sensor_cov.m / sec3_stats.m)
sensor_cov;

R = diag([R_p, R_q]);         % ruido de processo (gyro)
Q = diag([Q_phi, Q_theta]);   % ruido de medida (inclinometro)

sys = ss(A, [B eye(2)], C, 0);
[~, KALMAN_GAIN, ~] = kalman(sys, R, Q);   % kalman(sys, Qn=processo, Rn=medida): aqui R=processo, Q=medida

% constant
sampleTime = 1/200;
