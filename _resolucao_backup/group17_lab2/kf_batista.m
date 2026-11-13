%KF_BATISTA  Question 5: filter for the nominal system (2) of Batista, Silvestre, Oliveira (2011).
%   State x = [d; b], d = gravity direction in the body frame (unit vector, d = R'*e3),
%   b = rate-gyro bias (3). Dynamics d_dot = -S(w_m - b) d, b_dot = 0, output y = d (normalised
%   accelerometer, y = -f/|f|). Implemented as an EKF (Jacobian of the bilinear term), Ts ~ 1/77 s.
%   Run from LAB2/group17_lab2. Uses ../data/sensor_stats.mat (sec3_sensores) for Q/R.
clearvars -except sig_b rinf; close all;
dataDir = lab2_datadir();
figDir  = fullfile(fileparts(mfilename('fullpath')), 'figs');
relDir  = fullfile(fileparts(mfilename('fullpath')), '..', 'relatorio', 'figs');
if ~exist(figDir,'dir'), mkdir(figDir); end
S  = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
g0 = 9.80665;

st = load(fullfile(dataDir,'sensor_stats.mat')); st = st.stats;
sig_g = sqrt(diag(st.A.cov(4:6,4:6)))';           % gyro std at rest (A) [rad/s]
sig_a = sqrt(diag(st.C.cov(1:3,1:3)))';           % accel std in hover (C) [m/s^2]
Rm    = diag((sig_a/g0).^2);                      % measurement noise of d (normalised accel)
Sg    = diag(sig_g.^2);                           % gyro noise covariance
if ~exist("sig_b","var"), sig_b = 1e-5; end
if ~exist("rinf","var"), rinf = 25; end
Rm = rinf*Rm;
fprintf('sig_g = %s rad/s\nsig_a = %s m/s^2\n', mat2str(sig_g,3), mat2str(sig_a,3));

win = struct('A',[0 30],'C',[4.5 32],'D',[5 18.4]);
res = struct();
for e = {'A','C','D'}
    e = e{1};
    cf = fullfile(dataDir,['nd_' e '.mat']);
    if exist(cf,'file'), load(cf,'nd'); else, nd = load_navdata(fullfile(dataDir,['exp' e '.mat'])); save(cf,'nd'); end
    k  = find(nd.new & nd.t>=win.(e)(1) & nd.t<=win.(e)(2) & all(isfinite([nd.acc nd.gyr]),2));
    t = nd.t(k); w = nd.gyr(k,:); f = nd.acc(k,:);
    y = -f ./ vecnorm(f,2,2);                     % measured gravity direction
    N = numel(t);
    x = [y(1,:)'; 0;0;0];
    P = blkdiag(0.05^2*eye(3), (0.02)^2*eye(3));
    X = zeros(N,6); Pd = zeros(N,6); X(1,:) = x'; Pd(1,:) = diag(P)';
    for i = 2:N
        dt = t(i)-t(i-1);
        om = w(i-1,:)' - x(4:6);                  % unbiased rate
        % propagate d exactly (rotation by -om*dt in the body frame)
        th = norm(om)*dt;
        if th > 1e-9, K = S(om/norm(om)); Rot = eye(3) - sin(th)*K + (1-cos(th))*K*K;
        else, Rot = eye(3) - S(om)*dt; end
        d = x(1:3);
        A = [-S(om), -S(d); zeros(3,6)];          % Jacobian of [d_dot; b_dot]
        Phi = eye(6) + A*dt + (A*dt)^2/2;
        Qd  = S(d)*Sg*S(d)'*dt;                   % gyro noise enters d_dot as S(d)*n
        Q   = blkdiag(Qd, sig_b^2*dt*eye(3));
        x = [Rot*d; x(4:6)];
        P = Phi*P*Phi' + Q;
        % update
        H = [eye(3) zeros(3)];
        Kk = P*H'/(H*P*H'+Rm);
        x = x + Kk*(y(i,:)' - x(1:3));
        x(1:3) = x(1:3)/norm(x(1:3));
        P = (eye(6)-Kk*H)*P*(eye(6)-Kk*H)' + Kk*Rm*Kk';
        X(i,:) = x'; Pd(i,:) = diag(P)';
    end
    th_kf  = asin(max(-1,min(1,-X(:,1))))*180/pi;  phi_kf = atan2(X(:,2),X(:,3))*180/pi;
    th_raw = asin(max(-1,min(1,-y(:,1))))*180/pi;  phi_raw = atan2(y(:,2),y(:,3))*180/pi;
    res.(e) = struct('t',t,'X',X,'Pd',Pd,'th',th_kf,'phi',phi_kf,'th_raw',th_raw,'phi_raw',phi_raw, ...
                     'th_ob',nd.theta(k),'phi_ob',nd.phi(k),'w',w);
    n2 = t>t(1)+5;
    fprintf('\n=== Exp %s: final b_hat = %s rad/s | mean gyro = %s rad/s\n', e, mat2str(X(end,4:6),3), mat2str(mean(w),3));
    fprintf('   rmse vs onboard (t>%.0f s)  KF: th %.2f phi %.2f deg | raw: th %.2f phi %.2f deg\n', t(1)+5, ...
        rms(th_kf(n2)-res.(e).th_ob(n2)), rms(phi_kf(n2)-res.(e).phi_ob(n2)), ...
        rms(th_raw(n2)-res.(e).th_ob(n2)), rms(phi_raw(n2)-res.(e).phi_ob(n2)));
    fprintf('   std of KF angles: th %.2f phi %.2f | onboard th %.2f phi %.2f | raw th %.2f phi %.2f\n', ...
        std(th_kf(n2)), std(phi_kf(n2)), std(res.(e).th_ob(n2)), std(res.(e).phi_ob(n2)), std(th_raw(n2)), std(phi_raw(n2)));
    fprintf('   sqrt(P) bias final = %s rad/s\n', mat2str(sqrt(Pd(end,4:6)),3));

    % figures: angles and bias
    fig = figure('Position',[100 100 900 620]);
    subplot(3,1,1); plot(t,th_raw,'Color',[.75 .75 .75]); hold on; plot(t,res.(e).th_ob,'LineWidth',1); plot(t,th_kf,'LineWidth',1);
    ylabel('\theta [deg]'); grid on; legend('raw incl.','onboard','KF (Batista)','Location','best'); title(['Experiment ' e]);
    subplot(3,1,2); plot(t,phi_raw,'Color',[.75 .75 .75]); hold on; plot(t,res.(e).phi_ob,'LineWidth',1); plot(t,phi_kf,'LineWidth',1);
    ylabel('\phi [deg]'); grid on;
    subplot(3,1,3); plot(t,X(:,4:6),'LineWidth',1); hold on; set(gca,'ColorOrderIndex',1);
    plot(t([1 end]),[1;1]*mean(w),'--'); ylabel('\^b [rad/s]'); xlabel('t [s]'); grid on; legend('b_p','b_q','b_r');
    savepdf(fig, fullfile(figDir, sprintf('q5_exp%s.pdf',e)));
    copyfile(fullfile(figDir, sprintf('q5_exp%s.pdf',e)), relDir);
end
save(fullfile(dataDir,'q5_results.mat'),'res');
