function d = run_replay_5(e)
%RUN_REPLAY_5 runs ARDroneReplay_5 (section 5 filter) for experiment e ('A'..'E')
%   uses R, Q, x0, P0 from the workspace (run setup_5 first)
%   d.t [s], d.acc [mg], d.gyro [deg/s], d.phi_ob/theta_ob onboard [deg],
%   d.phi_m/theta_m inclinometer [deg], d.x estimated gravity [g],
%   d.phi/theta estimated [deg], d.b bias [deg/s], d.sb bias standard deviation [deg/s]

mdl  = 'ARDroneReplay_5';
here = fileparts(mfilename('fullpath'));
P = lab2_paths();
if ~bdIsLoaded(mdl), load_system(fullfile(here, [mdl '.slx'])); end

% experiment data
S = load(fullfile(P.data, ['exp' e '.mat']));
assignin('base', 'navdata', S.navdata);

% simulation
in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(S.navdata.time(end)));
[~, out] = evalc('sim(in)');          % evalc: the replay block prints "t = ..." at every step

% signals
d.t        = out.acc_log.time;
N          = numel(d.t);
d.acc      = fix2d(out.acc_log,  N);
d.gyro     = fix2d(out.gyro_log, N);
d.phi_ob   = squeeze(out.roll_onboard.signals.values);
d.theta_ob = squeeze(out.pitch_onboard.signals.values);
d.phi_m    = atan2d(-d.acc(:,2), -d.acc(:,3));
d.theta_m  = atan2d( d.acc(:,1), sqrt(d.acc(:,2).^2 + d.acc(:,3).^2));

% filter state -> angles and bias
xhat = fix2d(out.state_estimate, N);
[d.x, d.phi, d.theta, d.b, d.sb] = kf5_outputs(xhat, out.P_estimate.signals.values);
end

% N x channels
function X = fix2d(s, N)
X = squeeze(s.signals.values);
if size(X,1) ~= N, X = X.'; end
end
