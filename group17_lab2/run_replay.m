function d = run_replay(e)
%RUN_REPLAY runs ARDroneReplay_V2 (section 4 filter) for experiment e ('A'..'E')
%   uses A, B, C, KALMAN_GAIN from the workspace (run setup_4_1 or setup_4_4 first)
%   d.t [s], d.acc [mg], d.gyro [deg/s], d.phi_ob/theta_ob onboard [deg],
%   d.phi_m/theta_m inclinometer [deg], d.xhat filter state [deg, deg/s]

mdl  = 'ARDroneReplay_V2';
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
d.acc      = fix2d(out.acc_log,  numel(d.t));
d.gyro     = fix2d(out.gyro_log, numel(d.t));
d.phi_ob   = squeeze(out.roll_onboard.signals.values);
d.theta_ob = squeeze(out.pitch_onboard.signals.values);
d.phi_m    = atan2d(-d.acc(:,2), -d.acc(:,3));
d.theta_m  = atan2d( d.acc(:,1), sqrt(d.acc(:,2).^2 + d.acc(:,3).^2));
d.xhat     = fix2d(out.state_estimate, numel(d.t)) * 180/pi;
end

% N x channels
function X = fix2d(s, N)
X = squeeze(s.signals.values);
if size(X,1) ~= N, X = X.'; end
end
