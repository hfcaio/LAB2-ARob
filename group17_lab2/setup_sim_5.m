%% setup for the simulation of section 5 (ARDroneHoverSim_5)
% gyro_m = omega + bias + noise (rad/s, 3 axes)
% acc_m  = gravity in the body frame + noise (g, 3 axes, no linear accelerations)

setup_sim;      % drone model, manoeuvre, t, N, simDT, Tsim
setup_5;        % R_gyro, Q_acc, R, Q, x0, P0

% signals added to the measurements (From Workspace blocks), now with 3 axes
bias_true_val = [-0.03 0.05 0.04];      % [b_x b_y b_z] rad/s, artificial gyro bias
rng(17);                                % repeatable noise
noise_gyro = mkSignal5(t, randn(N,3) .* sqrt(diag(R_gyro).'));   % rad/s
noise_acc  = mkSignal5(t, randn(N,3) .* sqrt(diag(Q_acc).'));    % g
bias_gyro  = mkSignal5(t, repmat(bias_true_val, N, 1));

% signal in Structure With Time format
function s = mkSignal5(t, v)
s.time = t;
s.signals.values = v;
s.signals.dimensions = size(v, 2);
end
