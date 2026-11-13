%% Secção 3 — média e covariância dos sensores (exp. A, B, C) e inclinómetro (3.4)
%  Calcula nas janelas winStats (exp_windows.m) e, para comparação, no log inteiro.
%  Unidades do navdata: acelerómetro em mg, rate gyro em deg/s.

exp_windows;
setup_4_1;                          % o replay precisa de A, B, C, KALMAN_GAIN
stats = struct();

for e = 'ABC'
    d = run_replay(e);
    for full = [true false]
        if full, w = [d.t(1) d.t(end)]; tag = 'full'; else, w = winStats.(e); tag = 'win'; end
        m = d.t >= w(1) & d.t <= w(2);
        s = struct();  s.w = w;
        s.N        = sum(m);
        s.mu_a     = mean(d.acc(m,:));   s.S_a = cov(d.acc(m,:));
        s.mu_g     = mean(d.gyro(m,:));  s.S_g = cov(d.gyro(m,:));
        s.mu_incl  = [mean(d.phi_m(m)) mean(d.theta_m(m))];      % [roll pitch] deg
        s.std_incl = [std(d.phi_m(m))  std(d.theta_m(m))];
        stats.(e).(tag) = s;

        fprintf('\n=== Exp %s, %s [%g %g] s, N = %d\n', e, tag, w, s.N);
        fprintf('acc  mean [mg]    : % .2f % .2f % .2f\n', s.mu_a);
        fprintf('acc  std  [mg]    : % .3f % .3f % .3f\n', sqrt(diag(s.S_a)));
        fprintf('gyro mean [deg/s] : % .4f % .4f % .4f\n', s.mu_g);
        fprintf('gyro std  [deg/s] : % .4f % .4f % .4f\n', sqrt(diag(s.S_g)));
        disp('cov acc [mg^2]:');        disp(s.S_a);
        disp('cov gyro [(deg/s)^2]:');  disp(s.S_g);
        fprintf('inclinometer roll  mean %.4f std %.4f deg\n', s.mu_incl(1), s.std_incl(1));
        fprintf('inclinometer pitch mean %.4f std %.4f deg\n', s.mu_incl(2), s.std_incl(2));
    end
end

%% Q e R (hover, exp. C na janela) -> copiar para sensor_cov.m
s = stats.C.win;  k = (pi/180)^2;
fprintf('\nR_p = %.3g; R_q = %.3g;        %% (rad/s)^2\n', k*s.S_g(1,1), k*s.S_g(2,2));
fprintf('Q_phi = %.3g; Q_theta = %.3g;  %% rad^2\n',       k*s.std_incl(1)^2, k*s.std_incl(2)^2);

save(fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'sec3_stats.mat'), 'stats', 'winStats');
