%% plot 4.4: KF 4.4 (roll, pitch and bias) vs inclinometer and onboard, one pdf per experiment (4_4_expX.pdf)

exps    = {'A','B','C','D','E'};
SAVE    = true;                 % true -> save PDFs in saveDir
P = lab2_paths();  saveDir = P.figs;
if SAVE && ~exist(saveDir, 'dir'), mkdir(saveDir); end

exp_windows;                    % win: plot window [t_start t_end] in s
setup_4_4;                      % A, B, C, Q, R, KALMAN_GAIN
disp('KALMAN_GAIN ='); disp(KALMAN_GAIN);

for k = 1:numel(exps)
    e = exps{k};
    fprintf('Exp %s: simulating...\n', e);
    d = run_replay(e);          % d.xhat = [phi theta b_p b_q] in deg, deg/s
    m = d.t >= win.(e)(1) & d.t <= win.(e)(2);

    fig = figure('Name', ['4.4 - Exp ' e], 'Position', [100 100 900 750]);
    subplot(3,1,1);
    plot(d.t(m), d.phi_m(m), 'Color', [0.75 0.82 0.93]); hold on;
    plot(d.t(m), d.xhat(m,1), d.t(m), d.phi_ob(m), 'LineWidth', 1); grid on; xlim(win.(e));
    legend('inclinometer', 'Kalman (bias)', 'onboard'); ylabel('\phi [deg]'); title(['Roll - Exp ' e]);
    subplot(3,1,2);
    plot(d.t(m), d.theta_m(m), 'Color', [0.75 0.82 0.93]); hold on;
    plot(d.t(m), d.xhat(m,2), d.t(m), d.theta_ob(m), 'LineWidth', 1); grid on; xlim(win.(e));
    legend('inclinometer', 'Kalman (bias)', 'onboard'); ylabel('\theta [deg]'); title(['Pitch - Exp ' e]);
    subplot(3,1,3);
    plot(d.t(m), d.xhat(m,3), d.t(m), d.xhat(m,4), 'LineWidth', 1); grid on; xlim(win.(e));
    legend('b_x estimate', 'b_y estimate'); ylabel('bias [deg/s]'); xlabel('t [s]'); title(['Gyro bias estimate - Exp ' e]);

    if SAVE
        exportgraphics(fig, fullfile(saveDir, ['4_4_exp' e '.pdf']), 'ContentType', 'vector');
    end
end
