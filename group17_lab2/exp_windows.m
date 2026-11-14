%% time windows [t_start t_end] in s, per experiment
% win      -> plots (only cuts the end, where the log kept recording after landing)
% winStats -> section 3 statistics, Q and R, and RMS (steady state only)

% plots
win.A = [0 30];
win.B = [0 35];
win.C = [0 33];      % lands at ~32.7 s
win.D = [0 20];      % lands at ~18.4 s, handled after ~21 s
win.E = [0 35];      % lands at ~33 s, handled after ~41 s

% statistics
winStats.A = [10 24];   % disturbances at 0-3, 6-9 and 24-30 s
winStats.B = [10 33];   % 0-10 s: rotors spinning up and drone being held
winStats.C = [ 9 30];   % take-off until ~9 s, lands at ~32.7 s
winStats.D = [ 6 18];   % in flight
winStats.E = [ 8 32];   % in flight
