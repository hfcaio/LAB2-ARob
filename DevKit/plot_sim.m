%% Simulação: KF 4.1 e KF 4.4 vs ângulo verdadeiro, e bias vs bias estimado
%  ARDroneHoverSim_4_1 (sem bias) e ARDroneHoverSim_4_4 (com bias), mesmo bloco kalman_filter
%  Fig. 4_1_sim : modelo sem bias: pitch verdadeiro, inclinómetro e KF 4.1, e erro
%  Fig. 4_4_sim : modelo com bias: erro do KF 4.1 e do KF 4.4, e b_y vs b_y estimado para vários R_b

SAVE = true;
here    = fileparts(mfilename('fullpath'));
saveDir = fullfile(here, '..', 'relatorio', 'figs');
mdl41 = 'ARDroneHoverSim_4_1';     % sem bias no gyro
mdl44 = 'ARDroneHoverSim_4_4';     % com bias no gyro
for c = {mdl41, mdl44}
    if ~bdIsLoaded(c{1}), load_system(fullfile(here, 'simulation', [c{1} '.slx'])); end
end

setup_sim;                         % modelo do drone, ruído, bias
r2d = 180/pi;

%% 4.1: modelo sem bias, filtro da 4.1
setup_4_1;
[t, xt0, ym0, x0] = runSim(mdl41, Tsim);
m = t >= 20;
fprintf('RMS do erro para t >= 20 s [deg]      roll    pitch\n');
fprintf('  sem bias: inclinómetro            %7.3f %7.3f\n', rms(ym0(m,:) - xt0(m,:)));
fprintf('  sem bias: KF 4.1                  %7.3f %7.3f\n', rms(x0(m,:)  - xt0(m,:)));

%% 4.4: modelo com bias, filtro da 4.1 e depois o da 4.4
L_41 = KALMAN_GAIN;
[~, xt, ym, x41] = runSim(mdl44, Tsim);
setup_4_4;
[~, ~,  ~,  x44] = runSim(mdl44, Tsim);          % x44 = [phi theta b_p b_q]
fprintf('  com bias: KF 4.1                  %7.3f %7.3f\n', rms(x41(m,:)   - xt(m,:)));
fprintf('  com bias: KF 4.4                  %7.3f %7.3f\n', rms(x44(m,1:2) - xt(m,:)));
fprintf('offset médio do KF 4.1 [deg]: %.3f %.3f  (previsto b/L: %.3f %.3f)\n', ...
    mean(x41(m,:) - xt(m,:)), bias_true_val ./ diag(L_41).' * r2d);

%% Fig. 4_1_sim: só a 4.1 (modelo sem bias, KF 4.1), pitch
fig1 = figure('Name', 'sim 4.1', 'Visible', 'off', 'Position', [100 100 900 520]);
subplot(2,1,1);
plot(t, ym0(:,2), 'Color', [0.75 0.82 0.93]); hold on;
plot(t, x0(:,2), 'LineWidth', 1);
plot(t, xt0(:,2), 'k', 'LineWidth', 1);
grid on; ylabel('\theta [deg]');
legend('inclinometer', 'KF 4.1', 'true', 'Location', 'southeast', 'Orientation', 'horizontal');
subplot(2,1,2);
plot(t, ym0(:,2) - xt0(:,2), 'Color', [0.75 0.82 0.93]); hold on;
plot(t, x0(:,2) - xt0(:,2), 'LineWidth', 1);
grid on; ylabel('error [deg]'); xlabel('t [s]'); ylim([-2 2]);
legend('inclinometer', 'KF 4.1', 'Location', 'northeast', 'Orientation', 'horizontal');

%% varrimento de R_b: bias verdadeiro vs estimado
kbs  = [1e-3 1e-2 1e-1];
fig2 = figure('Name', 'sim 4.4', 'Visible', 'off', 'Position', [100 100 900 560]);
subplot(2,1,1);
plot(t, x41(:,2) - xt(:,2), t, x44(:,2) - xt(:,2), 'LineWidth', 1);
grid on; ylabel('\theta estimate - \theta [deg]'); ylim([-0.5 1.5]);
legend('KF 4.1', 'KF 4.4', 'Location', 'northeast', 'Orientation', 'horizontal');
subplot(2,1,2);
plot(t, bias_true_val(2)*ones(size(t))*r2d, 'k--', 'LineWidth', 1); hold on;
leg = {'$b_y$'};
for i = 1:numel(kbs)
    R44 = diag([R_p, R_q, kbs(i)*R_p, kbs(i)*R_q]);
    [~, KALMAN_GAIN] = kalman(sys, R44, Q);
    [tb, ~, ~, xb] = runSim(mdl44, Tsim);
    plot(tb, xb(:,4), 'LineWidth', 1);
    leg{end+1} = sprintf('$\\hat b_y$, $R_b = 10^{%d} R_\\omega$', round(log10(kbs(i)))); %#ok<SAGROW>
    mm = tb >= 45;
    fprintf('kb = %.0e: L = [%.3f; %.3f], erro do angulo RMS para t>=45 s = %.3f deg\n', ...
        kbs(i), KALMAN_GAIN(2,2), KALMAN_GAIN(4,2), rms(xb(mm,2) - xt(mm,2)));
end
setup_4_4;                         % repõe o ganho nominal
grid on; xlabel('t [s]'); ylabel('bias [deg/s]'); ylim([-1 5]);
legend(leg, 'Interpreter', 'latex', 'Location', 'southeast');

if SAVE
    exportgraphics(fig1, fullfile(saveDir, '4_1_sim.pdf'),      'ContentType', 'vector');
    exportgraphics(fig2, fullfile(saveDir, '4_4_sim.pdf'),      'ContentType', 'vector');
end

function [t, xt, ym, xh] = runSim(mdl, Tsim)
% corre o modelo com A, B, C, KALMAN_GAIN do base workspace; ângulos em deg, bias em deg/s
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(Tsim));
[~, out] = evalc('sim(in)');
t  = out.x_true.time;
xt = val(out.x_true)         * 180/pi;
ym = val(out.incl_m)         * 180/pi;
xh = val(out.state_estimate) * 180/pi;
end

function X = val(s)
X = squeeze(s.signals.values);
if size(X,1) ~= numel(s.time), X = X.'; end
end
