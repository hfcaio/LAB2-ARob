%% Janelas de tempo [t_ini t_fim] em s, por experimento
%  win      -> janela dos plots (corta só o manuseio depois do pouso)
%  winStats -> janela das estatísticas (secção 3, Q e R): só o regime permanente

win.A = [0 30];
win.B = [0 35];
win.C = [0 33];      % pouso em ~32.7 s
win.D = [0 20];      % pousa em ~18.4 s, manuseado depois de ~21 s
win.E = [0 35];      % pousa em ~33 s, manuseado depois de ~41 s

winStats.A = [10 24];   % perturbações em 0-3, 6-9 e 24-30 s
winStats.B = [10 33];   % 0-10 s: arranque dos rotores e mão a segurar
winStats.C = [ 9 30];   % descolagem até ~9 s, pouso em ~32.7 s
winStats.D = [ 6 18];   % em voo
winStats.E = [ 8 32];   % em voo
