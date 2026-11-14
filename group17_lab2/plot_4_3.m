%% plot 4.3: effect of Q and R on KF 4.1, pitch in exp. D and roll in exp. E (4_3_tuning.pdf)
% only the ratio R/Q matters, L = sqrt(R/Q): increasing Q (or decreasing R) lowers L

SAVE    = true;
P = lab2_paths();  saveDir = P.figs;
exp_windows;

kQ  = [1 1e2 1e4];              % Q -> kQ*Q
cfg = {'D', 2, 'theta_m', 'theta_ob', '\theta', [8 18];
       'E', 1, 'phi_m',   'phi_ob',   '\phi',   [8 30]};

% one run per value of Q
fig = figure('Name', '4.3', 'Position', [100 100 900 320]);
for c = 1:size(cfg,1)
    e = cfg{c,1};  leg = {'inclinometer'};
    subplot(1,2,c);
    for i = 1:numel(kQ)
        setup_4_1;
        [~, KALMAN_GAIN, ~] = kalman(sys, R, kQ(i)*Q);
        d = run_replay(e);
        m = d.t >= cfg{c,6}(1) & d.t <= cfg{c,6}(2);
        if i == 1, plot(d.t(m), d.(cfg{c,3})(m), 'Color', [0.75 0.82 0.93]); hold on; end
        plot(d.t(m), d.xhat(m,cfg{c,2}), 'LineWidth', 1);
        leg{end+1} = sprintf('L = %.2g rad/s', KALMAN_GAIN(cfg{c,2},cfg{c,2})); %#ok<SAGROW>
    end
    plot(d.t(m), d.(cfg{c,4})(m), 'k', 'LineWidth', 1);
    grid on; xlim(cfg{c,6}); xlabel('t [s]'); ylabel([cfg{c,5} ' [deg]']); title(['Exp ' e]);
    legend([leg {'onboard'}], 'Location', 'best', 'FontSize', 7);
end
setup_4_1;                      % restore the nominal gain

if SAVE
    exportgraphics(fig, fullfile(saveDir, '4_3_tuning.pdf'), 'ContentType', 'vector');
end
