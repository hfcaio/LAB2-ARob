%% plot 4.1: KF 4.1 (roll and pitch) vs inclinometer and onboard, one pdf per experiment (4_1_expX.pdf)

exps    = {'A','B','C','D','E'};
SAVE    = true;                 % true -> save PDFs in saveDir
P = lab2_paths();  saveDir = P.figs;
if SAVE && ~exist(saveDir, 'dir'), mkdir(saveDir); end

exp_windows;                    % win: plot window [t_start t_end] in s
setup_4_1;                      % A, B, C, Q, R, KALMAN_GAIN
fprintf('L_phi = %.3f, L_theta = %.3f rad/s\n', KALMAN_GAIN(1,1), KALMAN_GAIN(2,2));

for k = 1:numel(exps)
    e = exps{k};
    fprintf('Exp %s: simulating...\n', e);
    d = run_replay(e);
    m = d.t >= win.(e)(1) & d.t <= win.(e)(2);

    fig = figure('Name', ['4.1 - Exp ' e], 'Position', [100 100 900 600]);
    subplot(2,1,1);
    plot(d.t(m), d.phi_m(m), 'Color', [0.75 0.82 0.93]); hold on;
    plot(d.t(m), d.xhat(m,1), d.t(m), d.phi_ob(m), 'LineWidth', 1); grid on; xlim(win.(e));
    legend('inclinometer', 'Kalman', 'onboard'); ylabel('\phi [deg]'); title(['Roll - Exp ' e]);
    subplot(2,1,2);
    plot(d.t(m), d.theta_m(m), 'Color', [0.75 0.82 0.93]); hold on;
    plot(d.t(m), d.xhat(m,2), d.t(m), d.theta_ob(m), 'LineWidth', 1); grid on; xlim(win.(e));
    legend('inclinometer', 'Kalman', 'onboard'); ylabel('\theta [deg]'); xlabel('t [s]'); title(['Pitch - Exp ' e]);

    if SAVE
        exportgraphics(fig, fullfile(saveDir, ['4_1_exp' e '.pdf']), 'ContentType', 'vector');
    end
end
