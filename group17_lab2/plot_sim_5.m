%% simulation of section 5: ARDroneHoverSim_5 with artificial bias on the three gyros
% 5_sim.pdf: roll and pitch error, and true vs estimated bias with the +-1 sigma band of b_z

SAVE = true;
here    = fileparts(mfilename('fullpath'));
P = lab2_paths();  saveDir = P.figs;
mdl = 'ARDroneHoverSim_5';
if ~bdIsLoaded(mdl), load_system(fullfile(here, [mdl '.slx'])); end

clear kb kq
setup_sim_5;                       % drone model, noise, bias, R, Q, x0, P0
r2d = 180/pi;

% simulation
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(Tsim));
[~, out] = evalc('sim(in)');

t    = out.x_true.time;
eul  = val(out.x_true) * r2d;                     % true [phi theta psi], deg
ym   = val(out.acc_m);                            % measured gravity in the body frame, g
phi_m   = atan2d( ym(:,2), ym(:,3));
theta_m = atan2d(-ym(:,1), sqrt(ym(:,2).^2 + ym(:,3).^2));
[~, phi, theta, b, sb] = kf5_outputs(val(out.state_estimate), out.P_estimate.signals.values);
b_true = bias_true_val * r2d;

% errors and bias over time
m = t >= 20;
fprintf('max tilt [deg]: roll %.2f, pitch %.2f\n', max(abs(eul(:,1))), max(abs(eul(:,2))));
fprintf('RMS error for t >= 20 s [deg]         roll    pitch\n');
fprintf('  inclinometer                      %7.3f %7.3f\n', rms(phi_m(m)-eul(m,1)), rms(theta_m(m)-eul(m,2)));
fprintf('  KF 5                              %7.3f %7.3f\n', rms(phi(m)-eul(m,1)),   rms(theta(m)-eul(m,2)));
fprintf('true bias        [deg/s]: %7.3f %7.3f %7.3f\n', b_true);
for ts = [5 10 20 60]
    k = find(t >= ts, 1);
    fprintf('t = %2d s: b_hat = %7.3f %7.3f %7.3f   sigma = %6.3f %6.3f %6.3f deg/s\n', ts, b(k,:), sb(k,:));
end

% figure
fig = figure('Name', 'sim 5', 'Visible', 'off', 'Units', 'pixels', 'Position', [100 100 900 560]);
subplot(2,1,1);
plot(t, theta_m - eul(:,2), 'Color', [0.75 0.82 0.93]); hold on;
plot(t, phi - eul(:,1), t, theta - eul(:,2), 'LineWidth', 1);
grid on; ylabel('error [deg]'); ylim([-1.5 1.5]);
legend('inclinometer (pitch)', 'roll estimate', 'pitch estimate', 'Location', 'northeast', 'Orientation', 'horizontal');
subplot(2,1,2);
co = get(gca, 'ColorOrder');
h = gobjects(1,5);
for i = 1:3
    h(i) = plot(t, b(:,i), 'Color', co(i,:), 'LineWidth', 1); hold on;
    plot(t([1 end]), b_true(i)*[1 1], '--', 'Color', co(i,:));
end
h(4) = plot(nan, nan, 'k--');
h(5) = plot(t, b(:,3) + sb(:,3), ':', 'Color', co(3,:));
plot(t, b(:,3) - sb(:,3), ':', 'Color', co(3,:));
grid on; xlabel('t [s]'); ylabel('bias [deg/s]'); ylim([-4 6]);
legend(h, {'$\hat b_x$', '$\hat b_y$', '$\hat b_z$', 'true bias', '$\hat b_z \pm \sigma_{b_z}$'}, ...
    'Interpreter', 'latex', 'Location', 'northoutside', 'Orientation', 'horizontal');

if SAVE
    exportgraphics(fig, fullfile(saveDir, '5_sim.pdf'), 'ContentType', 'vector');
end

% N x channels
function X = val(s)
X = squeeze(s.signals.values);
if size(X,1) ~= numel(s.time), X = X.'; end
end
