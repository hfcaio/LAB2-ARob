%KF_ROLL  Question 4.7: steady-state Kalman filters for ROLL (same structure as pitch, 4.1-4.5).
%   Model 4.1 (no bias):  phi_dot = p_m + w,          y = phi_m + v
%   Model 4.4 (bias):     phi_dot = p_m - b + w,  b_dot = w_b,   y = phi_m + v
%   p_m = nd.gyr(:,1) (known input), phi_m = atan2(-ay,-az) (inclinometer), reference = nd.phi (onboard).
%   Filters run on FRESH navdata samples only (nd.new, ~66 Hz); gains from kalman() (steady state).
%   Needs ../data/sensor_stats.mat (run sec3_sensores first). Writes figs/q4_7_*.pdf and data/kf_roll_res.mat.

clear; close all;
dataDir = lab2_datadir();
figDir  = fullfile(fileparts(mfilename('fullpath')), 'figs');
load(fullfile(dataDir, 'sensor_stats.mat'));
g = 9.81;  d2r = pi/180;

% ---- load data (cache) --------------------------------------------------
E = {'A','B','C','D'};
win = struct('A',[0 30],'B',[8 30],'C',[8 30],'D',[5 18.4]);
for i = 1:4
    e = E{i};  nd = load_navdata(fullfile(dataDir, ['exp' e '.mat']));
    k = nd.new & nd.t >= win.(e)(1) & nd.t <= win.(e)(2);
    D.(e).t   = nd.t(k);  D.(e).p = nd.gyr(k,1);
    D.(e).phim = atan2(-nd.acc(k,2), -nd.acc(k,3));       % [rad]
    D.(e).phi  = nd.phi(k)*d2r;                           % onboard [rad]
end
Ts = median(diff(D.C.t));                                  % sample time of fresh packets
fprintf('Ts = %.4f s (%.1f Hz)\n', Ts, 1/Ts);

% ---- mounting offset (Exp A, at rest): inclinometer vs onboard ---------------
off = mean(D.A.phim - D.A.phi);
fprintf('Exp A: mean phi_m = %.3f deg, onboard = %.3f deg -> offset = %.3f deg; <ay> = %.4f m/s^2, |f| = %.3f m/s^2\n', ...
    mean(D.A.phim)/d2r, mean(D.A.phi)/d2r, off/d2r, stats.A.mean(2), norm(stats.A.mean(1:3)));
fprintf('  pure rotation of %.2f deg would give ay = %.3f m/s^2 and |f| = g = %.2f -> rest of ay offset is accelerometer bias/scale\n', ...
    off/d2r, -g*sin(off), g);
bgA = stats.A.mean(4);   fprintf('Exp A: gyro p mean (true gyro bias) = %.5f rad/s = %.3f deg/s\n', bgA, bgA/d2r);

% ---- noise parameters from sec3 covariances -----------------------------------
Qw0 = stats.A.cov(4,4);                      % gyro p noise variance at rest [rad^2/s^2]
Qwc = stats.C.cov(4,4);                      % in hover (includes vibration + motion)
R   = stats.C.cov(2,2)/g^2;                  % inclinometer variance in hover  [rad^2]  (d phi_m = d ay / g)
fprintf('var(p): A %.3e, C %.3e rad^2/s^2; R (C) = %.3e rad^2 (std %.3f deg)\n', Qw0, Qwc, R, sqrt(R)/d2r);

% ---- gains ----------------------------------------------------------------------
kf1 = @(Qw,R) gain1(Qw,R,Ts);   kf2 = @(Qw,Qb,R) gain2(Qw,Qb,R,Ts);
% tuning sweep on Exp C (hover): RMSE of filter vs onboard phi, for scaling kappa of Qw (Q/R)
kap = 10.^(0:0.5:5);  rm = zeros(size(kap));
for j = 1:numel(kap)
    x = run1(D.C.p, D.C.phim - off, kf1(kap(j)*Qw0, R), Ts);
    rm(j) = rms(x - D.C.phi)/d2r;
end
[~, jb] = min(rm);  kappa = kap(jb);
fprintf('Sweep on Exp C: kappa = %s\n  RMSE [deg] = %s\n -> kappa = %g\n', mat2str(kap,3), mat2str(rm,3), kappa);
Qw = kappa*Qw0;  Qb = 0.1*Qw;                % bias random walk, set relative to Qw (tuned in the bias test, Fig. q4_7_bias_test)
L1 = kf1(Qw, R);  L2 = kf2(Qw, Qb, R);
fprintf('4.1 L = %.5f  (pole %.4f, fc = %.3f Hz)\n', L1, 1-L1, -log(1-L1)/Ts/(2*pi));
fprintf('4.4 L = [%.5f %.5f]\n', L2);

% continuous-time gains equivalent: k = L/Ts
fprintf('equiv. continuous k = %.3f rad/s\n', L1/Ts);

% ---- evaluation on all experiments -------------------------------------------------
fprintf('\nRMSE vs onboard phi [deg]:  raw | raw-cal | KF4.1 raw | KF4.1 cal | KF4.4 raw | KF4.4 cal\n');
for i = 1:4
    e = E{i};  d = D.(e);  r = zeros(1,6);
    r(1) = rms(d.phim - d.phi);  r(2) = rms(d.phim - off - d.phi);
    [x1r, x2r] = runboth(d.p, d.phim, L1, L2, Ts);  [x1c, x2c, bc] = runboth(d.p, d.phim - off, L1, L2, Ts);
    r(3) = rms(x1r - d.phi); r(4) = rms(x1c - d.phi); r(5) = rms(x2r(:,1) - d.phi); r(6) = rms(x2c(:,1) - d.phi);
    fprintf('Exp %s: %s\n', e, sprintf('%7.3f ', r/d2r));
    res.(e) = struct('rmse', r/d2r, 'x1c', x1c, 'x2c', x2c(:,1), 'bc', x2c(:,2));
    mean_end.(e) = [mean(x1r(d.t>5)) mean(x2r(d.t>5,1)) mean(x2r(d.t>5,2)) mean(d.phim(d.t>5))];
end

% ---- Fig 1: Exp C hover (calibrated input) ---------------------------------------------
d = D.C;  [x1, x2] = runboth(d.p, d.phim - off, L1, L2, Ts);
f = figure('Position',[100 100 900 560]);
subplot(2,1,1); plot(d.t, (d.phim-off)/d2r, ':', 'Color',[.6 .6 .6]); hold on;
plot(d.t, x1/d2r, d.t, x2(:,1)/d2r, d.t, d.phi/d2r, 'k--', 'LineWidth', 1); grid on;
ylabel('\phi [deg]'); legend('\phi_m - offset','KF (4.1)','KF + bias (4.4)','onboard','Location','best'); title('Exp C (hover), roll');
subplot(2,1,2); plot(d.t, x2(:,2), 'LineWidth', 1); grid on; hold on; yline(bgA, 'r--', 'gyro mean (Exp A)');
ylabel('$\hat b_x$ [rad/s]', 'Interpreter','latex'); xlabel('t [s]');
savepdf(f, fullfile(figDir, 'q4_7_kf_expC.pdf'));

% ---- Fig 2: Exp A (rest): the mounting offset is NOT removed by the bias state --------------
d = D.A;  [x1, x2] = runboth(d.p, d.phim, L1, L2, Ts);                 % raw (uncalibrated)
[~, x2c] = runboth(d.p, d.phim - off, L1, L2, Ts);                     % calibrated
f = figure('Position',[100 100 900 560]);
subplot(2,1,1); plot(d.t, d.phim/d2r, ':', 'Color',[.6 .6 .6]); hold on;
plot(d.t, x1/d2r, d.t, x2(:,1)/d2r, d.t, x2c(:,1)/d2r, d.t, d.phi/d2r, 'k--', 'LineWidth', 1); grid on;
ylabel('\phi [deg]'); title('Exp A (rest): raw inclinometer has a mounting/accelerometer offset');
legend('\phi_m raw','KF (4.1)','KF + bias (4.4)','KF + bias, calibrated \phi_m','onboard','Location','southoutside','NumColumns',3,'FontSize',7);
subplot(2,1,2); plot(d.t, x2(:,2), d.t, x2c(:,2), 'LineWidth', 1); grid on; yline(bgA, 'r--');
ylabel('$\hat b_x$ [rad/s]', 'Interpreter','latex'); xlabel('t [s]'); legend('raw \phi_m','calibrated','gyro mean','FontSize',7);
savepdf(f, fullfile(figDir, 'q4_7_offset_expA.pdf'));
fprintf('\nExp A (t>5 s), mean: phi_m %.3f, KF4.1 %.3f, KF4.4 %.3f deg, bhat %.5f rad/s (gyro mean %.5f)\n', ...
    mean_end.A(4)/d2r, mean_end.A(1)/d2r, mean_end.A(2)/d2r, mean_end.A(3), bgA);

% ---- Fig 3: artificial gyro bias (4.4 suggestion), Exp C ----------------------------------------
b0 = 0.05;  d = D.C;  p_b = d.p + b0;                     % w_m = p + b
[x1b, x2b] = runboth(p_b, d.phim - off, L1, L2, Ts);
Qbs = Qw*[1e-3 1e-2 1e-1];  cols = lines(3);
f = figure('Position',[100 100 900 560]);
subplot(2,1,1); plot(d.t, d.phi/d2r, 'k--'); hold on; plot(d.t, x1b/d2r, 'Color',[.8 .2 .2]);
for j = 1:3, L2j = kf2(Qw, Qbs(j), R); [~, xj] = runboth(p_b, d.phim - off, L1, L2j, Ts);
    plot(d.t, xj(:,1)/d2r, 'Color', cols(j,:)); bh(:,j) = xj(:,2);
    fprintf('Qb/Qw=%g: L=[%.5f %.5f], RMSE phi %.3f deg, bhat(end-5s) = %.4f, std %.4f\n', Qbs(j)/Qw, L2j, ...
        rms(xj(:,1)-d.phi)/d2r, mean(xj(d.t>d.t(end)-5,2)), std(xj(d.t>d.t(end)-5,2))); end
fprintf('KF 4.1 with bias: RMSE %.3f deg (mean error %.3f deg)\n', rms(x1b-d.phi)/d2r, mean(x1b-d.phi)/d2r);
grid on; ylabel('\phi [deg]'); title('Exp C + artificial gyro bias b = 0.05 rad/s');
legend('onboard','KF 4.1','KF4.4  Q_b/Q_w=10^{-3}','10^{-2}','10^{-1}','Location','best');
subplot(2,1,2); plot(d.t, bh, 'LineWidth',1); hold on; yline(b0+bgA, 'k--', 'true b'); grid on;
ylabel('$\hat b_x$ [rad/s]','Interpreter','latex'); xlabel('t [s]');
savepdf(f, fullfile(figDir, 'q4_7_bias_test.pdf'));

% ---- Fig 4: complementary transfer functions (discrete, constant gains) -----------------------
A2 = [1 -Ts; 0 1];  B2 = [Ts; 0];  C2 = [1 0];
Ac = (eye(2) - L2*C2)*A2;  Bc = (eye(2) - L2*C2)*B2;
z = tf('z', Ts);
Gm1 = L1*z/(z - (1-L1));                                     % phi_m -> phi_hat (4.1), low-pass
Gp1 = Ts*(1-L1)/(z - (1-L1));                                % p_m  -> phi_hat (4.1)
Gm2 = z*tf(ss(Ac, L2, [1 0], 0, Ts));                        % phi_m -> phi_hat (4.4)
Gp2 = tf(ss(Ac, Bc, [1 0], 0, Ts));                          % p_m  -> phi_hat (4.4)
Int = Ts/(z - 1);                                            % integrator phi_gyro = Int*p_m
w = logspace(-2, log10(pi/Ts*0.9), 400);
H = @(G) squeeze(freqresp(G, w));
m1 = abs(H(Gm1));  m2 = abs(H(Gm2));
c1 = max(abs(H(Gm1) + H(Gp1)./H(Int) - 1));  c2 = max(abs(H(Gm2) + H(Gp2)./H(Int) - 1));
fprintf('Complementarity check max|Gm + Gp/Int - 1|: 4.1 %.2e, 4.4 %.2e\n', c1, c2);
f = figure('Position',[100 100 700 380]);
loglog(w/(2*pi), m1, w/(2*pi), m2, '--', w/(2*pi), abs(H(Gp1)./H(Int)), ':', w/(2*pi), abs(H(Gp2)./H(Int)), '-.'); grid on; ylim([1e-4 3]);
xlabel('f [Hz]'); ylabel('|G|'); legend('\phi_m \rightarrow \hat\phi (4.1)','\phi_m \rightarrow \hat\phi (4.4)','\phi_{gyro} \rightarrow \hat\phi (4.1)','\phi_{gyro} \rightarrow \hat\phi (4.4)','Location','east');
savepdf(f, fullfile(figDir, 'q4_7_tf_mag.pdf'));
fprintf('Corner: 4.1 fc=%.3f Hz; 4.4 |Gm|=-3dB at f=%.3f Hz\n', -log(1-L1)/Ts/(2*pi), interp1(m2(end:-1:1), w(end:-1:1), 1/sqrt(2))/(2*pi));
fprintf('poles 4.4: %s\n', mat2str(eig(Ac),4));
save(fullfile(dataDir, 'kf_roll_res.mat'), 'res', 'L1', 'L2', 'Ts', 'off', 'kappa', 'R', 'Qw', 'Qb');

% =========================== local functions =============================
function L = gain1(Qw, R, Ts)
    [~, ~, ~, L] = kalman(ss(1, Ts, 1, 0, Ts), Qw, R);
end
function L = gain2(Qw, Qb, R, Ts)
    [~, ~, ~, L] = kalman(ss([1 -Ts; 0 1], Ts*eye(2), [1 0], [0 0], Ts), diag([Qw Qb]), R);
end
function x = run1(p, y, L, Ts)
    x = zeros(size(y));  x(1) = y(1);
    for k = 2:numel(y), xp = x(k-1) + Ts*p(k-1);  x(k) = xp + L*(y(k) - xp); end
end
function [x1, x2, b] = runboth(p, y, L1, L2, Ts)
    n = numel(y);  x1 = run1(p, y, L1, Ts);  x2 = zeros(n,2);  x2(1,:) = [y(1) 0];
    A = [1 -Ts; 0 1];  B = [Ts; 0];
    for k = 2:n, xp = A*x2(k-1,:)' + B*p(k-1);  x2(k,:) = (xp + L2*(y(k) - xp(1)))'; end
    b = x2(:,2);
end
