%KF_PITCH_BIAS  Questions 4.4 and 4.5: pitch Kalman filter with gyro-bias state + complementarity.
%   Model 4.4: x = [theta; b_y],  theta_dot = (w_ym - b_y) + w,  b_dot = w_b,  y = theta_m + v
%   (w_ym = measured pitch rate = q + b_y, known input). Steady-state gain with kalman().
%   Own no-bias filter (4.1 model) is re-implemented here only for comparison.
%   Needs raw_incl.mat (run sec3_sensores first). Run from LAB2/group17_lab2.

clear; close all;
dataDir = lab2_datadir();
here    = fileparts(mfilename('fullpath'));
figDir  = fullfile(here, 'figs');            relDir = fullfile(here, '..', 'relatorio', 'figs');
if ~exist(figDir, 'dir'), mkdir(figDir); end
if ~exist(relDir, 'dir'), mkdir(relDir); end
load(fullfile(dataDir, 'raw_incl.mat'));
d2r = pi/180;

% ---- sample time: real packet period (~1/76 s), not the 200 Hz log rate -------------------
ndC = load_navdata(fullfile(dataDir, 'expC.mat'));
Ts  = mean(diff(ndC.t(ndC.new)));
fprintf('Ts (fresh packets) = %.5f s  (%.1f Hz)\n', Ts, 1/Ts);

% ---- tuning parameters (see report) ---------------------------------------------------------
sig_q = 0.03;          % gyro noise std in flight [rad/s]   (Exp C: std(q) = 0.0305; rest: 0.00075)
sig_y = 1.5*d2r;       % inclinometer noise std [rad]       (rest: 0.41 deg; larger in flight)
sig_b = 1e-3;         % bias random-walk intensity [rad/s/sqrt(s)]  <- chosen from simulation
% discrete noise covariances per step: theta increment (sig_q*Ts)^2 ; bias increment sig_b^2*Ts
gains = @(sb) design(Ts, sig_q, sig_y, sb);
[L1, L2] = gains(sig_b);
fprintf('4.1 (no bias)  L = %.5f   -> k ~ %.3f 1/s\n', L1, L1/Ts);
fprintf('4.4 (bias)     L = [%.5f %.5f]\n', L2);
% cross-check of kalman() with the Riccati equation
A2 = [1 -Ts; 0 1];  C2 = [1 0];  Qd = diag([(sig_q*Ts)^2, sig_b^2*Ts]);
P  = idare(A2', C2', Qd, sig_y^2);  Lchk = P*C2'/(C2*P*C2' + sig_y^2);
% (P is the a-priori covariance, so Lchk is the current-estimator gain)
fprintf('    Riccati check: L = [%.5f %.5f]\n', Lchk);

% ---- continuous-time equivalent gains (for 4.5 / transfer functions) -------------------------
% PSDs: q_c = sig_q^2*Ts , r_c = sig_y^2*Ts , qb_c = sig_b^2
Ac = [0 -1; 0 0]; Cc = [1 0];
sysc = ss(Ac, [1 0;0 1], Cc, 0);  Kc = lqe(Ac, eye(2), Cc, diag([sig_q^2*Ts, sig_b^2]), sig_y^2*Ts);
k1 = Kc(1); k2 = -Kc(2);          % theta_hat' = w - b_hat + k1 e ;  b_hat' = -k2 e
kc = sig_q/sig_y;                  % 1-state continuous gain
fprintf('continuous: k = %.3f (1-state);  k1 = %.3f, k2 = %.4f  (wn = %.3f rad/s, zeta = %.2f)\n', ...
        kc, k1, k2, sqrt(k2), k1/(2*sqrt(k2)));
fprintf('discrete/Ts: k1 = %.3f, k2 = %.4f\n', L2(1)/Ts, L2(2)/Ts*-1);

% ============================ 1) Simulation with artificial bias ============================
rng(1);  T = 60;  t = (0:Ts:T)'; N = numel(t);
th_true = 5*d2r*(sin(2*pi*0.15*t) + 0.6*sin(2*pi*0.4*t));        % rad
q_true  = [diff(th_true)/Ts; 0];
b_true  = 0.05*ones(N,1);  b_true(t >= 30) = 0.08;                % artificial bias [rad/s]
wym     = q_true + b_true + sig_q*randn(N,1);                     % gyro measurement
ysim    = th_true + sig_y*randn(N,1);                             % inclinometer
sbs = [1e-4 3e-4 1e-3 1e-2];
cols = lines(numel(sbs));
fig = figure('Position', [100 100 900 700]);
subplot(3,1,1); plot(t, b_true, 'k', 'LineWidth', 1.6); hold on;
res = struct();
for i = 1:numel(sbs)
    [~, Li] = gains(sbs(i));
    X = runKF(t, wym, ysim, Li, true, [ysim(1); 0]);
    plot(t, X(:,2), 'Color', cols(i,:), 'LineWidth', 1);
    res(i).rmsth = rms(X(t>5,1) - th_true(t>5))/d2r;
    res(i).rmsb  = rms(X(t>5 & t<30,2) - b_true(t>5 & t<30));
    ix = find(abs(X(:,2)-b_true) < 0.1*0.05, 1);                  % first time within 10% of b
    res(i).t90 = t(ix);
    fprintf('sim sigma_b=%.0e: L=[%.4f %.5f] t(10%%)=%.1f s, rms(theta err)=%.3f deg, rms(b err)=%.4f rad/s\n', ...
            sbs(i), Li, res(i).t90, res(i).rmsth, res(i).rmsb);
end
ylabel('$b_y,\ \hat b_y$ [rad/s]', 'Interpreter', 'latex'); grid on; xlim([0 T]);
legend(['true b_y', arrayfun(@(s) sprintf('\\sigma_b = %.1e', s), sbs, 'UniformOutput', false)], ...
       'Location', 'southeast', 'NumColumns', 3); title('Simulation: artificial bias +0.05 rad/s (step to +0.08 at 30 s)');
% chosen tuning: theta error with and without bias state
X2 = runKF(t, wym, ysim, L2, true,  [ysim(1); 0]);
X1 = runKF(t, wym, ysim, L1, false, ysim(1));
% naive: 1-state filter fed with biased gyro AND a version where the gyro bias is known and removed
subplot(3,1,2); plot(t, (ysim - th_true)/d2r, ':', 'Color', [.6 .6 .6]); hold on;
plot(t, (X1 - th_true)/d2r, 'LineWidth', 1.2); plot(t, (X2(:,1) - th_true)/d2r, 'LineWidth', 1.2);
ylabel('\theta error [deg]'); grid on; xlim([0 T]); legend('raw inclinometer', 'KF no bias', 'KF with bias', 'Location', 'northeast');
subplot(3,1,3); plot(t, (X2(:,2) - b_true), 'LineWidth', 1.2); grid on; xlim([0 T]);
ylabel('$\hat b_y - b_y$ [rad/s]', 'Interpreter', 'latex'); xlabel('t [s]');
savepdf(fig, fullfile(figDir, 'q4_4_sim_bias.pdf'));
fprintf('sim, chosen: rms th err  raw %.3f | no-bias %.3f (mean %.3f) | bias %.3f deg\n', ...
        rms(ysim(t>5)-th_true(t>5))/d2r, rms(X1(t>5)-th_true(t>5))/d2r, mean(X1(t>5)-th_true(t>5))/d2r, ...
        rms(X2(t>5,1)-th_true(t>5))/d2r);

% ============================ 2) Real data =================================================
sel = struct('A', [0.5 29.5], 'C', [4.5 32], 'D', [5 18.4]);   % usable windows
out = struct();
for e = {'A','C','D'}
    e = e{1};
    nd = load_navdata(fullfile(dataDir, ['exp' e '.mat']));
    m  = nd.new & nd.t >= sel.(e)(1) & nd.t <= sel.(e)(2) & ~isnan(nd.gyr(:,2)) & ~isnan(raw.(e).theta_m);
    tt = nd.t(m);  q = nd.gyr(m,2);  y = raw.(e).theta_m(m)*d2r;  ob = nd.theta(m)*d2r;
    Xa = runKF(tt, q, y, L1, false, y(1));
    Xb = runKF(tt, q, y, L2, true,  [y(1); 0]);
    out.(e) = struct('t', tt, 'y', y, 'ob', ob, 'X1', Xa, 'X2', Xb, 'q', q);
    w = tt > tt(1) + 5;                                             % skip initial transient
    fprintf('Exp %s [%.1f-%.1f s]: mean q = %.5f rad/s | b_hat(end) = %.5f | std b_hat(last half) = %.5f\n', ...
            e, tt(1), tt(end), mean(q), Xb(end,2), std(Xb(tt > mean(tt([1 end])),2)));
    fprintf('   rms diff to onboard [deg]: raw %.3f | KF no bias %.3f | KF bias %.3f   (std onboard %.3f)\n', ...
            rms(y(w)-ob(w))/d2r, rms(Xa(w)-ob(w))/d2r, rms(Xb(w,1)-ob(w))/d2r, std(ob(w))/d2r);
    fprintf('   mean offset to onboard [deg]: raw %.3f | KF no bias %.3f | KF bias %.3f\n', ...
            mean(y(w)-ob(w))/d2r, mean(Xa(w)-ob(w))/d2r, mean(Xb(w,1)-ob(w))/d2r);
end

% --- sensitivity of the real-data results to sigma_b ---
fprintf('Real-data sweep over sigma_b (rms diff to onboard [deg]; std of b_hat in 2nd half [rad/s]):\n');
for sb = [3e-4 1e-3 3e-3 1e-2]
    [~, Ls] = gains(sb);  str = sprintf('  sb=%.1e L=[%.4f %.5f]', sb, Ls);
    for e = {'A','C','D'}
        o = out.(e{1});  Xs = runKF(o.t, o.q, o.y, Ls, true, [o.y(1); 0]);  w = o.t > o.t(1)+5;
        str = [str sprintf(' | %s: rms %.3f, std b %.4f, b_end %.4f', e{1}, rms(Xs(w,1)-o.ob(w))/d2r, std(Xs(o.t>mean(o.t([1 end])),2)), Xs(end,2))]; %#ok<AGROW>
    end
    disp(str);
end

% --- Exp A: bias convergence at rest ---
o = out.A;
fig = figure('Position', [100 100 900 420]);
plot(o.t, o.X2(:,2), 'LineWidth', 1.4); hold on;
yline(mean(o.q), 'k--', 'LineWidth', 1.2);
legend('$\hat b_y$', sprintf('mean gyro $q$ (%.4f rad/s)', mean(o.q)), 'Interpreter', 'latex', 'Location', 'southeast');
xlabel('t [s]'); ylabel('rad/s'); grid on; title('Exp A (at rest): bias estimate');
savepdf(fig, fullfile(figDir, 'q4_4_expA_bias.pdf'));
ix = find(abs(o.X2(:,2) - mean(o.q)) < 1e-3, 1);
fprintf('Exp A: b_hat within 1 mrad/s of mean q after %.1f s; mean b_hat last 10 s = %.5f, std = %.5f; mean q = %.5f\n', ...
        o.t(ix)-o.t(1), mean(o.X2(o.t>o.t(end)-10,2)), std(o.X2(o.t>o.t(end)-10,2)), mean(o.q));

% --- Exp C and D ---
for e = {'C','D'}
    e = e{1}; o = out.(e);
    fig = figure('Position', [100 100 900 560]);
    subplot(3,1,[1 2]);
    plot(o.t, o.y/d2r, 'Color', [.75 .75 .75]); hold on;
    plot(o.t, o.ob/d2r, 'k', 'LineWidth', 1.2);
    plot(o.t, o.X1/d2r, 'LineWidth', 1.1); plot(o.t, o.X2(:,1)/d2r, 'LineWidth', 1.1);
    ylabel('\theta [deg]'); grid on; xlim(o.t([1 end])');
    legend('raw inclinometer', 'onboard', 'KF no bias', 'KF with bias', 'Location', 'best');
    title(sprintf('Experiment %s', e));
    subplot(3,1,3); plot(o.t, o.X2(:,2), 'LineWidth', 1.2); yline(mean(o.q), 'k--');
    ylabel('$\hat b_y$ [rad/s]', 'Interpreter', 'latex'); xlabel('t [s]'); grid on; xlim(o.t([1 end])');
    savepdf(fig, fullfile(figDir, sprintf('q4_4_exp%s.pdf', e)));
end

% ============================ 3) Question 4.5: complementarity =============================
s = tf('s');
F1_th = kc/(s+kc);            F1_w = 1/(s+kc);                 % 1 state:  theta_hat = F_th*theta_m + F_w*w_ym
F2_th = (k1*s+k2)/(s^2+k1*s+k2);  F2_w = s/(s^2+k1*s+k2);       % 2 states
S1 = minreal(F1_th + s*F1_w);  S2 = minreal(F2_th + s*F2_w);
fprintf('4.5: minreal(F_th + s*F_w): 1-state order %d, dcgain %.3f ; 2-state order %d, dcgain %.3f\n', order(S1), dcgain(S1), order(S2), dcgain(S2));
% discrete check (exact): sum over frequency grid
z = tf('z', Ts);  Iz = Ts/(z-1);
Fd1_th = L1*z/(z-(1-L1));  Fd1_w = (1-L1)*Ts/(z-(1-L1));       % current estimator, forward Euler
fprintf('   discrete 1-state: max|F_th + F_w/I - 1| = %.2e\n', max(abs(squeeze(freqresp(Fd1_th + Fd1_w/Iz - 1, logspace(-2, log10(pi/Ts*0.99), 50))))));
% numerical 2-state discrete complementary check via state-space of the estimator
Ae = (eye(2) - L2(:)*C2)*A2;  Be_y = L2(:);  Be_w = (eye(2) - L2(:)*C2)*[Ts; 0];
Fd2 = @(zz) [1 0]*((zz*eye(2) - Ae)\[Be_y*zz, Be_w]);           % [F_th(z) F_w(z)] (current-estimator form)
wg = logspace(-2, log10(pi/Ts*0.99), 50);  dev = zeros(size(wg));
for i = 1:numel(wg)
    zz = exp(1j*wg(i)*Ts);  F = Fd2(zz);  dev(i) = abs(F(1) + F(2)*(zz-1)/(Ts) - 1);
end
fprintf('   discrete 2-state: max|F_th + F_w*(z-1)/Ts - 1| = %.2e (forward-Euler derivative)\n', max(dev));

% steady-state error for a constant gyro bias b (wym = q + b):  1-state: b/k ; 2-state: 0
fprintf('   constant bias error: 1-state b/k = %.4f * b rad  (b=0.05 -> %.2f deg);  2-state: 0\n', 1/kc, 0.05/kc/d2r);

w = logspace(-2, 2, 400);
fig = figure('Position', [100 100 900 420]);
subplot(1,2,1);
semilogx(w, 20*log10(abs(squeeze(freqresp(F1_th, w)))), w, 20*log10(abs(squeeze(freqresp(s*F1_w, w)))), '--', 'LineWidth', 1.4);
hold on; xline(kc, ':'); grid on; ylim([-60 5]); xlabel('\omega [rad/s]'); ylabel('|.| [dB]');
legend('F_{\theta_m} (LP)', 's F_{w}  (HP)', 'Location', 'southwest'); title('No bias: k = \sigma_q/\sigma_y');
subplot(1,2,2);
semilogx(w, 20*log10(abs(squeeze(freqresp(F2_th, w)))), w, 20*log10(abs(squeeze(freqresp(s*F2_w, w)))), '--', 'LineWidth', 1.4);
hold on; xline(sqrt(k2), ':'); grid on; ylim([-60 5]); xlabel('\omega [rad/s]');
legend('F_{\theta_m}', 's F_{w}', 'Location', 'southwest'); title('With bias: (k_1 s+k_2)/(s^2+k_1 s+k_2)');
savepdf(fig, fullfile(figDir, 'q4_5_complementary.pdf'));

% copy all figures of this script to the report folder
for f = {'q4_4_sim_bias','q4_4_expA_bias','q4_4_expC','q4_4_expD','q4_5_complementary'}
    copyfile(fullfile(figDir, [f{1} '.pdf']), fullfile(relDir, [f{1} '.pdf']));
end

% ====================================== local functions ======================================
function [L1, L2] = design(Ts, sig_q, sig_y, sig_b)
% steady-state (current-estimator) Kalman gains, 1- and 2-state, via kalman()
sys1 = ss(1, [Ts 1], 1, [0 0], Ts);                           % inputs [w_ym (known), w]
[~, ~, ~, L1] = kalman(sys1, (sig_q*Ts)^2, sig_y^2);
A2 = [1 -Ts; 0 1];  B2 = [Ts; 0];
sys2 = ss(A2, [B2 eye(2)], [1 0], zeros(1,3), Ts);            % inputs [w_ym (known), w, w_b]
[~, ~, ~, L2] = kalman(sys2, diag([(sig_q*Ts)^2, sig_b^2*Ts]), sig_y^2);
L2 = L2(:)';
end

function X = runKF(t, u, y, L, withBias, x0)
% x_k|k-1 = F x_{k-1|k-1} + [dt;0] u(k-1) ;  x_k|k = x_k|k-1 + L (y - theta_pred)
% actual packet interval dt is used in the prediction, gain fixed at the nominal Ts value
n = numel(t);  ns = 1 + withBias;  X = zeros(n, ns);  X(1,:) = x0(:)';
for k = 2:n
    dt = t(k) - t(k-1);
    if withBias, xp = [X(k-1,1) + dt*(u(k-1) - X(k-1,2)); X(k-1,2)];
    else,        xp = X(k-1,1) + dt*u(k-1);
    end
    X(k,:) = (xp + L(:)*(y(k) - xp(1)))';
end
end
