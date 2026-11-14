%% plot 4.5: bode (magnitude only) of the pitch complementary filters (4_5_bode.pdf)
% KF 4.1: G_L = L/(s+L),                    G_H = s/(s+L)
% KF 4.4: G_L = (L1 s - L2)/(s^2+L1 s-L2),  G_H = s^2/(s^2+L1 s-L2)
P = lab2_paths();  saveDir = P.figs;

s = tf('s');
w = logspace(-3, 2, 500);

% pitch gains
setup_4_1;  L  = KALMAN_GAIN(2,2);
setup_4_4;  L1 = KALMAN_GAIN(2,2);  L2 = KALMAN_GAIN(4,2);  % pitch and bias
fprintf('4.1: L = %.4f | 4.4: L1 = %.4f, L2 = %.4f, poles = %s\n', L, L1, L2, mat2str(roots([1 L1 -L2]).', 4));

% filters and check of G_L + G_H = 1
G_L  = L/(s+L);                      G_H  = s/(s+L);
G_Lb = (L1*s - L2)/(s^2 + L1*s - L2);  G_Hb = s^2/(s^2 + L1*s - L2);
fprintf('max |G_L+G_H-1| = %.2g, max |G_Lb+G_Hb-1| = %.2g\n', ...
    max(abs(squeeze(freqresp(G_L+G_H, w)) - 1)), max(abs(squeeze(freqresp(G_Lb+G_Hb, w)) - 1)));

% figure
fig = figure;
opts = bodeoptions;
opts.PhaseVisible = 'off';
opts.Title.String = '';
opts.Grid = 'on';
bodeplot(G_L, G_H, G_Lb, '--', G_Hb, '--', minreal(G_L + G_H), 'k:', w, opts);
ylim([-80 10]);
legend('G_L (4.1)', 'G_H (4.1)', 'G_L (4.4)', 'G_H (4.4)', 'G_L + G_H', 'Location', 'southeast');
set(fig, 'Position', [100 100 700 350]);

exportgraphics(fig, fullfile(saveDir, '4_5_bode.pdf'), 'ContentType', 'vector');
