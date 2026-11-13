%% setup file for LAB2 — simulação da secção 5 (ARDroneHoverSim_5)
%  O modelo calcula, a partir dos ângulos de Euler do simulador e das suas derivadas,
%    gyro_m = omega + bias + ruído          (rad/s, 3 eixos)
%    acc_m  = gravidade no corpo + ruído    (g, 3 eixos; sem acelerações lineares)
%  e usa o mesmo bloco kalman_filter do ARDroneReplay_5 (R, Q, x0, P0 de setup_5).

setup_sim;      % modelo do drone, manobra (degraus em x e y), t, N, simDT, Tsim
setup_5;        % R_gyro, Q_acc, R, Q, x0, P0

% --- sinais somados às medidas (From Workspace), agora com 3 eixos
bias_true_val = [-0.03 0.05 0.04];      % [b_x b_y b_z] rad/s, bias artificial do gyro
rng(17);                                % ruído repetível
noise_gyro = mkSignal5(t, randn(N,3) .* sqrt(diag(R_gyro).'));   % rad/s
noise_acc  = mkSignal5(t, randn(N,3) .* sqrt(diag(Q_acc).'));    % g
bias_gyro  = mkSignal5(t, repmat(bias_true_val, N, 1));

function s = mkSignal5(t, v)
s.time = t;
s.signals.values = v;
s.signals.dimensions = size(v, 2);
end
