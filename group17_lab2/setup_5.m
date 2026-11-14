%% setup 5: time-varying Kalman filter for partial attitude + bias of the 3 gyros (Batista et al. 2011)
% system (2): x_dot = -S(w_m) x - S(y) b,  b_dot = 0,  y = x
% x = gravity in the body frame (g), b = bias (rad/s), filter state = [x; b]
% used by ARDroneReplay_5 and ARDroneHoverSim_5 (kalman_filter block)

d2r = pi/180;

% uncertainty (exp. C in hover, 9-30 s, from sec3_stats.m)
R_gyro = diag([1.4469 1.5830 1.5971]*d2r).^2;   % (rad/s)^2  var(gyro x, y, z)
Q_acc  = diag([7.6897 6.5676 7.5346]*1e-3).^2;  % g^2        var(acc x, y, z)

% tuning (can be set before running this setup)
if ~exist('kb', 'var') || isempty(kb), kb = 1e-4; end   % R_bias = kb*R_gyro
if ~exist('kq', 'var') || isempty(kq), kq = 1;    end   % Q = kq*Q_acc, kq > 1 trusts the accelerometer less

R = blkdiag(R_gyro, kb*R_gyro);   % process noise [gyro; bias], enters through G(t) = blkdiag(-S(y), I)
Q = kq*Q_acc;                     % measurement noise

% initial conditions: level drone, unknown bias (sigma 0.05 rad/s = 2.9 deg/s)
x0 = [0; 0; 1; 0; 0; 0];
P0 = blkdiag(1e-3*eye(3), 0.05^2*eye(3));

% constant
sampleTime = 1/200;
