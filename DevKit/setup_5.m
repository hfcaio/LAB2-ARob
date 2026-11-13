%% setup file for LAB2 — 5 (atitude parcial + bias dos 3 rate gyros, Batista et al. 2011)
%  Sistema nominal (2) na forma LTV, com x = gravidade no corpo (g) e b = bias (rad/s):
%    x_dot = -S(w_m) x - S(y) b       b_dot = 0       y = x
%  Estado do filtro: state_hat = [x; b] (6x1). Entradas do bloco kalman_filter:
%    gyro_m (rad/s) e acc_m = -acelerómetro (g). Parâmetros: R, Q, x0, P0.
%  Convenção do grupo: R = ruído de processo (gyro), Q = ruído de medida (acelerómetro).

d2r = pi/180;

% uncertainty (exp. C em hover, 9-30 s, valores de sec3_stats.m)
R_gyro = diag([1.4469 1.5830 1.5971]*d2r).^2;   % (rad/s)^2  var(gyro x, y, z)
Q_acc  = diag([7.6897 6.5676 7.5346]*1e-3).^2;  % g^2        var(acc x, y, z)

% afinação
if ~exist('kb', 'var') || isempty(kb), kb = 1e-4; end   % R_bias = kb*R_gyro (passeio aleatório do bias)
if ~exist('kq', 'var') || isempty(kq), kq = 1;    end   % Q = kq*Q_acc (kq > 1: confia menos no acelerómetro)

R = blkdiag(R_gyro, kb*R_gyro);   % ruido de processo: [gyro; bias], entra por G(t) = blkdiag(-S(y), I)
Q = kq*Q_acc;                     % ruido de medida

% condições iniciais: drone nivelado, bias desconhecido (sigma 0.05 rad/s = 2.9 deg/s)
x0 = [0; 0; 1; 0; 0; 0];
P0 = blkdiag(1e-3*eye(3), 0.05^2*eye(3));

% constant
sampleTime = 1/200;
