%% plot 5: section 5 filter in experiments A-E vs inclinometer, onboard and KF 4.4 (measured Q and 100 Q)
% 5_angles.pdf: pitch in A, C and D, roll in E
% 5_bias.pdf: estimated bias of the three gyros in A, C, D and E
% prints the RMS difference to the onboard estimate and the bias in the winStats window

exps    = {'A','B','C','D','E'};
SAVE    = true;
P = lab2_paths();  saveDir = P.figs;
exp_windows;

% replay with the three filters
res = struct();
for k = 1:numel(exps)
    e = exps{k};
    fprintf('Exp %s: simulating...\n', e);
    setup_4_4;                 d4 = run_replay(e);       % KF 4.4 (section 4)
    clear kb kq;      setup_5; d  = run_replay_5(e);     % KF 5, measured Q
    kq = 100;         setup_5; dq = run_replay_5(e);     % KF 5, 100 Q
    d.xhat44 = d4.xhat;
    d.phi_q = dq.phi;  d.theta_q = dq.theta;  d.b_q = dq.b;  d.sb_q = dq.sb;
    res.(e) = d;
end
clear kb kq;  setup_5;         % restore the nominal tuning

%% RMS difference to the onboard estimate and estimated bias
fprintf('\nRMS difference to onboard [deg], winStats window\n');
fprintf('exp |  roll: incl   KF4.4  KF5    KF5(100Q) | pitch: incl   KF4.4  KF5    KF5(100Q)\n');
for k = 1:numel(exps)
    e = exps{k};  d = res.(e);
    m = d.t >= winStats.(e)(1) & d.t <= winStats.(e)(2);
    fprintf(' %s  |       %6.3f %6.3f %6.3f %6.3f    |        %6.3f %6.3f %6.3f %6.3f\n', e, ...
        rms(d.phi_m(m)-d.phi_ob(m)),     rms(d.xhat44(m,1)-d.phi_ob(m)),   rms(d.phi(m)-d.phi_ob(m)),     rms(d.phi_q(m)-d.phi_ob(m)), ...
        rms(d.theta_m(m)-d.theta_ob(m)), rms(d.xhat44(m,2)-d.theta_ob(m)), rms(d.theta(m)-d.theta_ob(m)), rms(d.theta_q(m)-d.theta_ob(m)));
end
fprintf('\nbias [deg/s], winStats window, kq = 1: mean, max |b|, final sigma; and gyro mean\n');
for k = 1:numel(exps)
    e = exps{k};  d = res.(e);
    m = d.t >= winStats.(e)(1) & d.t <= winStats.(e)(2);  kf = find(m, 1, 'last');
    fprintf(' %s  | mean %6.3f %6.3f %6.3f | max %5.2f %5.2f %5.2f | sigma %5.3f %5.3f %5.3f | gyro mean %6.3f %6.3f %6.3f | KF4.4 %6.3f %6.3f\n', e, ...
        mean(d.b(m,:)), max(abs(d.b(m,:))), d.sb(kf,:), mean(d.gyro(m,:)), mean(d.xhat44(m,3:4)));
end

%% angles
cfg = {'A', 'theta'; 'C', 'theta'; 'D', 'theta'; 'E', 'phi'};
lab = struct('theta', '\theta', 'phi', '\phi');
col = struct('theta', 2, 'phi', 1);
fig1 = figure('Name', '5_angles', 'Visible', 'off', 'Units', 'pixels', 'Position', [100 100 900 600]);
tl = tiledlayout(fig1, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:size(cfg,1)
    e = cfg{c,1};  a = cfg{c,2};  d = res.(e);
    m = d.t >= win.(e)(1) & d.t <= win.(e)(2);
    nexttile(tl);
    plot(d.t(m), d.([a '_m'])(m), 'Color', [0.75 0.82 0.93]); hold on;
    plot(d.t(m), d.xhat44(m,col.(a)), d.t(m), d.(a)(m), d.t(m), d.([a '_q'])(m), 'LineWidth', 1);
    plot(d.t(m), d.([a '_ob'])(m), 'k', 'LineWidth', 1);
    grid on; xlim(win.(e)); title(['Exp ' e]); ylabel([lab.(a) ' [deg]']);
    ys = sort(d.([a '_m'])(m));                      % ignore the landing spikes
    yl = ys(round([0.005 0.995]*(numel(ys)-1)) + 1).';
    yl = [min([yl(1); d.([a '_ob'])(m)]) max([yl(2); d.([a '_ob'])(m)])];
    ylim(yl + [-0.1 0.1]*diff(yl));
    if c >= 3, xlabel('t [s]'); end
end
lg = legend('inclinometer', 'KF 4.4', 'KF 5', 'KF 5 (100 Q)', 'onboard', 'Orientation', 'horizontal');
lg.Layout.Tile = 'north';

%% bias
fig2 = figure('Name', '5_bias', 'Visible', 'off', 'Units', 'pixels', 'Position', [100 100 900 420]);
tl = tiledlayout(fig2, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:size(cfg,1)
    e = cfg{c,1};  d = res.(e);
    m = d.t >= win.(e)(1) & d.t <= win.(e)(2);
    nexttile(tl);
    plot(d.t(m), d.b(m,:), 'LineWidth', 1);
    grid on; xlim(win.(e)); ylim([-1 1]); title(['Exp ' e]); ylabel('bias [deg/s]');
    if c >= 3, xlabel('t [s]'); end
end
lg = legend({'$\hat b_x$', '$\hat b_y$', '$\hat b_z$'}, 'Interpreter', 'latex', 'Orientation', 'horizontal');
lg.Layout.Tile = 'north';
fprintf('fig1 %s, fig2 %s px\n', mat2str(fig1.Position), mat2str(fig2.Position));

if SAVE
    exportgraphics(fig1, fullfile(saveDir, '5_angles.pdf'), 'ContentType', 'vector');
    exportgraphics(fig2, fullfile(saveDir, '5_bias.pdf'),   'ContentType', 'vector');
end
