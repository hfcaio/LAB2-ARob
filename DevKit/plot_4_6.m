%% Plot 4.6 / 4.7 — resumo dos experimentos A-E: KF 4.1 e KF 4.4 vs inclinómetro e onboard
%  Gera 4_6_pitch.pdf e 4_7_roll.pdf (um painel por experimento) e imprime o RMS
%  da diferença para o onboard na janela winStats.

exps    = {'A','B','C','D','E'};
SAVE    = true;
saveDir = fullfile(fileparts(mfilename('fullpath')), '..', 'relatorio', 'figs');
exp_windows;

res = struct();
for k = 1:numel(exps)
    e = exps{k};
    fprintf('Exp %s: simulando...\n', e);
    setup_4_1;  d  = run_replay(e);
    setup_4_4;  d4 = run_replay(e);
    d.xhat44 = d4.xhat;
    res.(e)  = d;
end

%% RMS para o onboard e bias estimado (janela winStats)
fprintf('\nRMS da diferenca para o onboard [deg] e bias medio estimado [deg/s], janela winStats\n');
fprintf('exp |  roll: incl   KF4.1  KF4.4 | pitch: incl   KF4.1  KF4.4 |   b_x     b_y\n');
for k = 1:numel(exps)
    e = exps{k};  d = res.(e);
    m = d.t >= winStats.(e)(1) & d.t <= winStats.(e)(2);
    fprintf(' %s  |       %6.3f %6.3f %6.3f |        %6.3f %6.3f %6.3f | %6.3f  %6.3f\n', e, ...
        rms(d.phi_m(m)-d.phi_ob(m)),     rms(d.xhat(m,1)-d.phi_ob(m)),   rms(d.xhat44(m,1)-d.phi_ob(m)), ...
        rms(d.theta_m(m)-d.theta_ob(m)), rms(d.xhat(m,2)-d.theta_ob(m)), rms(d.xhat44(m,2)-d.theta_ob(m)), ...
        mean(d.xhat44(m,3)), mean(d.xhat44(m,4)));
end

%% figuras resumo
cfg = {'4_6_pitch', 'theta_m', 2, 'theta_ob', '\theta';
       '4_7_roll',  'phi_m',   1, 'phi_ob',   '\phi'};
for c = 1:size(cfg,1)
    fig = figure('Name', cfg{c,1}, 'Position', [100 100 900 800]);
    for k = 1:numel(exps)
        e = exps{k};  d = res.(e);
        m = d.t >= win.(e)(1) & d.t <= win.(e)(2);
        subplot(3,2,k);
        plot(d.t(m), d.(cfg{c,2})(m), 'Color', [0.75 0.82 0.93]); hold on;
        plot(d.t(m), d.xhat(m,cfg{c,3}), d.t(m), d.xhat44(m,cfg{c,3}), 'LineWidth', 1);
        plot(d.t(m), d.(cfg{c,4})(m), 'k', 'LineWidth', 1);
        grid on; xlim(win.(e)); title(['Exp ' e]); ylabel([cfg{c,5} ' [deg]']);
        ys = sort(d.(cfg{c,2})(m));                      % ignora os picos do pouso
        yl = ys(round([0.005 0.995]*(numel(ys)-1)) + 1).';
        yl = [min([yl(1); d.(cfg{c,4})(m)]) max([yl(2); d.(cfg{c,4})(m)])];
        ylim(yl + [-0.1 0.1]*diff(yl));
        if k >= 4, xlabel('t [s]'); end
    end
    lg = legend('inclinometer', 'KF 4.1', 'KF 4.4 (bias)', 'onboard');
    lg.Position(1:2) = [0.62 0.16];
    if SAVE
        exportgraphics(fig, fullfile(saveDir, [cfg{c,1} '.pdf']), 'ContentType', 'vector');
    end
end
