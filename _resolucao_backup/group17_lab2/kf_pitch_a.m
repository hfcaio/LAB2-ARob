%KF_PITCH_A  Questions 4.1-4.3: steady-state Kalman filter for pitch (no gyro bias).
%   Model:  theta_dot = q_m + w ,  y = theta_m + v   (gyro q_m = known input)
%   Discrete (Ts = 1/76.5 s, rate of fresh navdata packets):
%       theta[k+1] = theta[k] + Ts*q_m[k] + Ts*w[k],   y[k] = theta[k] + v[k]
%   Gain from kalman() (Control Toolbox).  Run from LAB2/group17_lab2.
%   Outputs: figs/q4_1_*.pdf, q4_2_bode.pdf, q4_3_*.pdf (+ copies in ../relatorio/figs).

clear; close all;
dataDir = lab2_datadir();
figDir  = fullfile(fileparts(mfilename('fullpath')), 'figs');
repDir  = fullfile(fileparts(mfilename('fullpath')), '..', 'relatorio', 'figs');
if ~exist(figDir,'dir'), mkdir(figDir); end
if ~exist(repDir,'dir'), mkdir(repDir); end
sv = @(f,n) cellfun(@(d) savepdf(f, fullfile(d, n)), {figDir, repDir});
d2r = pi/180;  r2d = 180/pi;

%% ---- data (fresh packets only, cached because load_navdata is slow) ----
cache = fullfile(dataDir, 'kfa_cache.mat');
if exist(cache,'file'), load(cache);
else
  for e = 'CD'
    nd = load_navdata(fullfile(dataDir, ['exp' e '.mat']));
    S.(e) = struct('t',nd.t,'theta',nd.theta,'q',nd.gyr(:,2),'acc',nd.acc,'new',nd.new);
  end
  save(cache,'S');
end
win.C = [8 28];  win.D = [5 18.4];
load(fullfile(dataDir,'sensor_stats.mat'));          % stats.A/B/C
D = struct();
for e = 'CD'
  s = S.(e);  m = s.new & s.t>=win.(e)(1) & s.t<=win.(e)(2);
  th_m = atan2(s.acc(:,1), sqrt(s.acc(:,2).^2 + s.acc(:,3).^2));      % rad
  D.(e) = struct('t',s.t(m),'q',s.q(m),'y',th_m(m),'onb',s.theta(m)*d2r);
end

%% ---- choice of Q and R ------------------------------------------------
Ts = mean(diff(D.C.t));                               % ~ 1/76.5 s
sig_q_A = sqrt(stats.A.cov(5,5));  sig_q_C = sqrt(stats.C.cov(5,5));   % rad/s
% Gyro: use the in-flight (hover) value -- vibration dominates; Exp A (motors off) is 40x smaller.
Qw = sig_q_C^2;                                       % [rad^2/s^2]
% Inclinometer: A (rest) gives sigma ~0.34 deg but in flight the accelerometer measures specific
% force; effective error = theta_m - onboard theta over C and D (onboard = reference).
res = [D.C.y-D.C.onb; D.D.y-D.D.onb];
R  = var(res - mean(res));                            % [rad^2]
fprintf('sigma_q: A %.4f, C %.4f rad/s | sigma_theta(flight) %.3f deg (rest 0.34 deg)\n', ...
        sig_q_A, sig_q_C, sqrt(R)*r2d);

%% ---- 4.1 gain with kalman() --------------------------------------------
gain = @(Qw,R) gainfun(Qw,R,Ts);
[M, P] = gain(Qw,R);
a  = 1 - M;                                           % closed-loop pole
k_eq = -log(a)/Ts;                                    % equivalent continuous k [rad/s]
fprintf('4.1: M = %.5f, pole %.5f, k = %.3f rad/s (fc = %.3f Hz), sqrt(Qw/R) = %.3f\n', ...
        M, a, k_eq, k_eq/2/pi, sqrt(Qw/R));
fprintf('     steady-state posterior std = %.3f deg\n', sqrt((1-M)*P)*r2d);

%% ---- offline filter on real data (real dt between packets) --------------
kf = @(d,M) runkf(d.t, d.q, d.y, M);
for e = 'CD'
  D.(e).kf = kf(D.(e), M);
  % reference "onboard" is itself an estimate -> remove mean offset (inclinometer bias ~0.5-0.9 deg)
  err = @(x) sqrt(mean(((x-D.(e).onb) - mean(x-D.(e).onb)).^2))*r2d;
  fprintf('Exp %s: RMS dev. from onboard [deg]: raw %.3f | KF %.3f | std(KF) %.3f, std(raw) %.3f, std(onb) %.3f\n', ...
          e, err(D.(e).y), err(D.(e).kf), std(D.(e).kf)*r2d, std(D.(e).y)*r2d, std(D.(e).onb)*r2d);
end
fig = figure('Position',[100 100 900 640]);
for i = 1:2
  ee = 'CD'; e = ee(i);  d = D.(e);
  subplot(2,1,i); plot(d.t,d.y*r2d,'Color',[.75 .75 .75]); hold on;
  plot(d.t,d.onb*r2d,'--','LineWidth',1.2); plot(d.t,d.kf*r2d,'LineWidth',1.4); grid on;
  ylabel('\theta [deg]'); tt = {'hover','pitch pulse'}; title(sprintf('Exp %s (%s)', e, tt{i}));
  if i==1, legend('raw \theta_m','onboard','KF (4.1)','Location','northeast','NumColumns',3); end
end
xlabel('t [s]'); sv(fig,'q4_1_exp.pdf');

%% ---- synthetic simulation ------------------------------------------------
rng(1);  Tsim = 30;  t = (0:Ts:Tsim)';  n = numel(t);
th = 8*d2r*( (t>3&t<8) - (t>14&t<18) ) ;                          % rectangular pulses
th = filter(1-exp(-Ts/0.25), [1 -exp(-Ts/0.25)], th) + 2*d2r*sin(2*pi*0.3*t).*(t>20); % smooth
q_true = gradient(th,Ts);
qm = q_true + sqrt(Qw)*randn(n,1);           % gyro: white noise (no bias in 4.1)
ym = th + sqrt(R)*randn(n,1);                % inclinometer
th_int = th(1) + cumsum([0; qm(1:end-1)])*Ts; % open-loop integration of the gyro
th_kf  = runkf(t, qm, ym, M);
rms = @(x) sqrt(mean((x-th).^2))*r2d;
fprintf('Synthetic RMS error [deg]: raw %.3f | gyro integration %.3f | KF %.3f (theory post. %.3f)\n', ...
        rms(ym), rms(th_int), rms(th_kf), sqrt((1-M)*P)*r2d);
fig = figure('Position',[100 100 900 560]);
subplot(2,1,1); plot(t,ym*r2d,'Color',[.8 .8 .8]); hold on; plot(t,th*r2d,'k','LineWidth',1.2);
plot(t,th_kf*r2d,'LineWidth',1.4); plot(t,th_int*r2d,'--'); grid on; ylabel('\theta [deg]');
legend('y=\theta_m','true','KF','gyro integration','Location','northwest','NumColumns',2);
subplot(2,1,2); plot(t,(th_kf-th)*r2d); hold on; plot(t,(th_int-th)*r2d,'--'); grid on;
ylabel('error [deg]'); xlabel('t [s]'); legend('KF','gyro integration');
sv(fig,'q4_1_sim.pdf');

%% ---- 4.2 transfer functions (continuous k and exact discrete) -----------
s = tf('s');  Tth = k_eq/(s+k_eq);  Tw = 1/(s+k_eq);
z = tf('z',Ts);
Tth_d = M*z/(z-a);                    % theta_hat/theta_m (current estimator)
Tw_d  = a*Ts/(z-a);                   % theta_hat/q_m  : theta_hat[k]=a*(theta_hat[k-1]+Ts*q[k-1]) + M*y
fprintf('4.2: T_theta = %.3f/(s+%.3f); T_w = 1/(s+%.3f); DC: %.3f, %.3f\n', k_eq,k_eq,k_eq, dcgain(Tth), dcgain(Tw*s));
fig = figure('Position',[100 100 900 420]);
w = logspace(-2, log10(pi/Ts), 400);
Tint = Tw*s;                           % gyro path as angle estimate per integrated gyro: s/(s+k)
subplot(1,2,1);
loglog(w, abs(squeeze(freqresp(Tth,w))), w, abs(squeeze(freqresp(Tint,w))), 'LineWidth',1.4); hold on;
loglog(w, abs(squeeze(freqresp(Tth_d,w))), 'k:', 'LineWidth',1);
xline(k_eq,'--'); grid on; xlabel('\omega [rad/s]'); ylabel('magnitude');
legend('k/(s+k) : \theta_m\rightarrow\hat\theta','s/(s+k) : \theta_{gyro}\rightarrow\hat\theta','discrete','Location','southwest');
subplot(1,2,2);
loglog(w, abs(squeeze(freqresp(Tw,w))), w, abs(squeeze(freqresp(Tw_d,w))), 'k:', 'LineWidth',1.2); hold on;
loglog(w, 1./w, 'r--'); xline(k_eq,'--'); grid on; xlabel('\omega [rad/s]');
ylabel('magnitude'); legend('1/(s+k) : q_m\rightarrow\hat\theta','discrete','1/s (pure integrator)','Location','southwest');
sv(fig,'q4_2_bode.pdf');

%% ---- 4.3 effect of Q/R ----------------------------------------------------
ratio = logspace(-4, 4, 33);           % Q/R in [1/s^2]  (Qw/R)
kk = zeros(size(ratio));  MM = kk;  Pp = kk;
for i = 1:numel(ratio)
  [MM(i), Pp(i)] = gain(ratio(i)*R, R);  kk(i) = -log(1-MM(i))/Ts;
end
fprintf('Nominal Q/R = %.3f 1/s^2\n', Qw/R);
tab = [0.01 0.1 1 Qw/R 10 100 1000]; 
fprintf('   Q/R     M       k[rad/s]  fc[Hz]  post.std/sqrt(R)\n');
for r_ = tab
  [m_, p_] = gain(r_*R, R);  k_ = -log(1-m_)/Ts;
  fprintf('%9.3f %8.5f %9.3f %7.3f %8.3f\n', r_, m_, k_, k_/2/pi, sqrt((1-m_)*p_/R));
end
fig = figure('Position',[100 100 900 380]);
subplot(1,2,1); loglog(ratio, kk, 'LineWidth',1.4); hold on; loglog(ratio, sqrt(ratio), 'k:');
loglog(Qw/R, k_eq, 'ro', 'MarkerFaceColor','r'); grid on;
xlabel('Q/R [s^{-2}]'); ylabel('k [rad/s]'); legend('kalman()','k=\surd(Q/R)','nominal','Location','northwest');
subplot(1,2,2); semilogx(ratio, MM, 'LineWidth',1.4);; hold on;
semilogx(Qw/R, M, 'ro', 'MarkerFaceColor','r'); grid on; xlabel('Q/R [s^{-2}]'); ylabel('gain M'); ylim([0 1]);
sv(fig,'q4_3_gain.pdf');

% response comparison: tuning with Q/R x0.01, x1, x100 on Exp D and synthetic
mult = [0.01 1 100];  names = {'Q/R \times 0.01','nominal','Q/R \times 100'};
fig = figure('Position',[100 100 900 600]);  col = lines(3);
for i = 1:3
  Mi = gain(mult(i)*Qw, R);  ki = -log(1-Mi)/Ts;
  xD = runkf(D.D.t, D.D.q, D.D.y, Mi);  xS = runkf(t, qm, ym, Mi);
  eD = xD - D.D.onb;  eD = eD - mean(eD);
  fprintf('mult %6.2f: M=%.4f k=%.3f | Exp D dev from onboard %.3f deg, std %.3f | synth RMS %.3f deg\n', ...
          mult(i), Mi, ki, sqrt(mean(eD.^2))*r2d, std(xD)*r2d, rms(xS));
  subplot(2,1,1); hold on; plot(D.D.t, xD*r2d, 'Color', col(i,:), 'LineWidth',1.2);
  subplot(2,1,2); hold on; plot(t, (xS-th)*r2d, 'Color', col(i,:), 'LineWidth',1.1);
end
subplot(2,1,1); plot(D.D.t, D.D.onb*r2d, 'k--'); grid on; ylabel('\theta [deg]'); title('Exp D');
legend([names {'onboard'}],'Location','northeast','NumColumns',2);
subplot(2,1,2); grid on; ylabel('synthetic error [deg]'); xlabel('t [s]');
sv(fig,'q4_3_resp.pdf');

%% ---- local functions ------------------------------------------------------
function [M, P] = gainfun(Qw, R, Ts)
  sys = ss(1, [Ts Ts], 1, [0 0], Ts);             % inputs [q_m ; w]; w enters through Ts
  [~, ~, P, M] = kalman(sys, Qw, R);             % M: innovation gain (current estimator), P prior var
end

function x = runkf(t, q, y, M)
  N = numel(t);  x = zeros(N,1);  x(1) = mean(y(1:min(20,N)));
  for k = 2:N
    xp = x(k-1) + (t(k)-t(k-1))*q(k-1);           % predict with real dt
    x(k) = xp + M*(y(k) - xp);
  end
end
