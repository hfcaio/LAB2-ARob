function d = run_replay(e)
%RUN_REPLAY  Corre o ARDroneReplay_V2 para o experimento e ('A'..'E').
%   Usa A, B, C, KALMAN_GAIN e sampleTime do base workspace (setup_4_1 ou
%   setup_4_4). Devolve
%     d.t        tempo [s]
%     d.acc      acelerómetro [mg]      (N×3)
%     d.gyro     rate gyro [deg/s]      (N×3)
%     d.phi_ob, d.theta_ob   estimativa onboard [deg]
%     d.phi_m,  d.theta_m    inclinómetro [deg] (mesma convenção do KF)
%     d.xhat     estado do KF (N×n): [phi theta] em deg, bias (se n=4) em deg/s

mdl  = 'ARDroneReplay_V2';
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
if ~bdIsLoaded(mdl), load_system(fullfile(here, 'Replay', [mdl '.slx'])); end

S = load(fullfile(here, '..', 'data', ['exp' e '.mat']));
assignin('base', 'navdata', S.navdata);

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(S.navdata.time(end)));
[~, out] = evalc('sim(in)');          % evalc: o bloco de replay imprime "t = ..." a cada passo

d.t        = out.acc_log.time;
d.acc      = fix2d(out.acc_log,  numel(d.t));
d.gyro     = fix2d(out.gyro_log, numel(d.t));
d.phi_ob   = squeeze(out.roll_onboard.signals.values);
d.theta_ob = squeeze(out.pitch_onboard.signals.values);
d.phi_m    = atan2d(-d.acc(:,2), -d.acc(:,3));
d.theta_m  = atan2d( d.acc(:,1), sqrt(d.acc(:,2).^2 + d.acc(:,3).^2));
d.xhat     = fix2d(out.state_estimate, numel(d.t)) * 180/pi;
end

function X = fix2d(s, N)
X = squeeze(s.signals.values);
if size(X,1) ~= N, X = X.'; end
end
