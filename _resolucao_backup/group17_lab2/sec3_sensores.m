%SEC3_SENSORES  Questions 3.1-3.4 of Lab 2: sensor statistics and raw inclinometer.
%   Run from LAB2/group17_lab2 (data in ../data). Produces figs/ and prints tables.

clear; close all;
dataDir = lab2_datadir();
figDir  = fullfile(fileparts(mfilename('fullpath')), 'figs');
if ~exist(figDir, 'dir'), mkdir(figDir); end

% Time windows to analyse [s] -- check the plots and adjust.
%   A: at rest            B: rotors on, held on the ground   C: hover
%   D: pitch step (drone lands at ~18.5 s; after ~21 s it was handled -> discard)
% A: bumps of the table at 0-3, 6-9 and 24-30 s (std(ax) doubles) -> use the quiet 10-24 s.
% B: 0-9 s = spin-up and grabbing the drone; C: take-off transient until ~9 s, landing after ~30 s.
win.A = [10 24];  win.B = [10 33];  win.C = [9 30];  win.D = [6 18];

exps = {'A','B','C'};
for k = 1:numel(exps)
    e  = exps{k};
    nd = load_navdata(fullfile(dataDir, ['exp' e '.mat']));
    m  = nd.new & nd.t >= win.(e)(1) & nd.t <= win.(e)(2);
    Z  = [nd.acc(m,:) nd.gyr(m,:)];
    fprintf('\n=== Experiment %s  (%d fresh samples, %.1f-%.1f s)\n', e, sum(m), win.(e));
    fprintf('mean acc  [m/s^2] : % .4f % .4f % .4f\n', mean(Z(:,1:3)));
    fprintf('std  acc  [m/s^2] : % .4f % .4f % .4f\n', std(Z(:,1:3)));
    fprintf('mean gyro [rad/s] : % .5f % .5f % .5f\n', mean(Z(:,4:6)));
    fprintf('std  gyro [rad/s] : % .5f % .5f % .5f\n', std(Z(:,4:6)));
    stats.(e).mean = mean(Z);  stats.(e).cov = cov(Z);
    disp('Covariance [ax ay az p q r]:'); disp(stats.(e).cov);
    disp('Correlation:'); disp(corrcoef(Z));
    if e == 'A'     % sign check: accelerometer only sees gravity, motors off
        thm = atan2(nd.acc(:,1), sqrt(nd.acc(:,2).^2+nd.acc(:,3).^2))*180/pi;
        phm = atan2(-nd.acc(:,2), -nd.acc(:,3))*180/pi;
        kk = nd.new & nd.t>=1; r1 = corrcoef(thm(kk), nd.theta(kk)); r2 = corrcoef(phm(kk), nd.phi(kk));
        fprintf('A, whole log: corr(theta_raw,theta_ob)=%.2f  corr(phi_raw,phi_ob)=%.2f (positive => SGN=+1)\n', r1(1,2), r2(1,2));
    end
end
save(fullfile(dataDir, 'sensor_stats.mat'), 'stats', 'win');

% --- 3.4 raw inclinometer --------------------------------------------------
% Specific force in body frame (z down) at rest: f = g*[ sin(th); -sin(phi)cos(th); -cos(phi)cos(th) ]
%   theta_m = atan2( fx, sqrt(fy^2+fz^2) ),   phi_m = atan2( -fy, -fz )
% CONFIRM THE SIGNS with a hand-tilt test (see PLANO_LAB2.md): if the raw angle has
% the opposite sign of nd.theta / nd.phi (onboard estimate), flip SGN_TH / SGN_PHI.
SGN_TH = 1;  SGN_PHI = 1;
win.D = [5 18.4];                       % D: only the usable part (see PLANO_LAB2.md)
win.A = [0 30];
for e = {'A','B','C','D'}
    e  = e{1};
    nd = load_navdata(fullfile(dataDir, ['exp' e '.mat']));
    f  = nd.acc;
    th_m  = SGN_TH  * atan2(f(:,1), sqrt(f(:,2).^2 + f(:,3).^2)) * 180/pi;
    phi_m = SGN_PHI * atan2(-f(:,2), -f(:,3)) * 180/pi;
    k = nd.new & nd.t >= win.(e)(1) & nd.t <= win.(e)(2);
    if e == 'B' || e == 'C', k = nd.new & nd.t >= 3 & nd.t <= 30; end
    fig = figure('Name', ['exp' e], 'Position', [100 100 640 400], 'Color', 'w');
    tl = tiledlayout(2,1,'TileSpacing','compact','Padding','compact');
    nexttile; plot(nd.t(k), th_m(k), 'Color', [0.70 0.80 0.95], 'LineWidth', 0.6); hold on;
    plot(nd.t(k), nd.theta(k), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
    ylabel('\theta [deg]'); grid on; legend('raw (accelerometer)', 'onboard', 'Location', 'best');
    nexttile; plot(nd.t(k), phi_m(k), 'Color', [0.70 0.80 0.95], 'LineWidth', 0.6); hold on;
    plot(nd.t(k), nd.phi(k), 'Color', [0.85 0.33 0.10], 'LineWidth', 1.2);
    ylabel('\phi [deg]'); xlabel('t [s]'); grid on;
    set(fig, 'Position', [100 100 640 400]); drawnow;
    savepdf(fig, fullfile(figDir, sprintf('q3_4_raw_exp%s.pdf', e)));
    exportgraphics(fig, fullfile('/tmp', sprintf('prev_%s.png', e)), 'Resolution', 110);
    raw.(e) = struct('t', nd.t, 'theta_m', th_m, 'phi_m', phi_m, 'new', nd.new);
end
save(fullfile(dataDir, 'raw_incl.mat'), 'raw');

% --- 3.4 numbers: raw vs onboard, same windows as the plots -----------------
win.A = [10 24]; win.B = [10 33]; win.C = [9 30]; win.D = [6 18];
for e = {'A','B','C','D'}
    e = e{1}; nd = load_navdata(fullfile(dataDir, ['exp' e '.mat']));
    f = nd.acc; k = nd.new & nd.t >= win.(e)(1) & nd.t <= win.(e)(2);
    th_m  = SGN_TH  * atan2(f(:,1), sqrt(f(:,2).^2 + f(:,3).^2)) * 180/pi;
    phi_m = SGN_PHI * atan2(-f(:,2), -f(:,3)) * 180/pi;
    c1 = corrcoef(th_m(k), nd.theta(k)); c2 = corrcoef(phi_m(k), nd.phi(k));
    fprintf('%s: theta raw mean %.2f std %.2f | onboard mean %.2f std %.2f | corr %.2f | rms diff %.2f\n', e, mean(th_m(k)), std(th_m(k)), mean(nd.theta(k)), std(nd.theta(k)), c1(1,2), rms(th_m(k)-nd.theta(k)));
    fprintf('%s: phi   raw mean %.2f std %.2f | onboard mean %.2f std %.2f | corr %.2f | rms diff %.2f\n', e, mean(phi_m(k)), std(phi_m(k)), mean(nd.phi(k)), std(nd.phi(k)), c2(1,2), rms(phi_m(k)-nd.phi(k)));
end
