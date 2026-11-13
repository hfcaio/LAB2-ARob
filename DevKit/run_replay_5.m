function d = run_replay_5(e)
%RUN_REPLAY_5  Corre o ARDroneReplay_5 (filtro da secção 5) para o experimento e ('A'..'E').
%   Usa R, Q, x0, P0 e sampleTime do base workspace (setup_5). Devolve
%     d.t        tempo [s]
%     d.acc      acelerómetro [mg]      (N×3)
%     d.gyro     rate gyro [deg/s]      (N×3)
%     d.phi_ob, d.theta_ob   estimativa onboard [deg]
%     d.phi_m,  d.theta_m    inclinómetro [deg]
%     d.x        gravidade estimada no corpo [g]    (N×3)
%     d.phi, d.theta         roll e pitch estimados [deg]
%     d.b        bias estimado [deg/s]              (N×3)
%     d.sb       desvio padrão do bias, sqrt(diag(P)) [deg/s]  (N×3)

mdl  = 'ARDroneReplay_5';
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
if ~bdIsLoaded(mdl), load_system(fullfile(here, 'Replay', [mdl '.slx'])); end

S = load(fullfile(here, '..', 'data', ['exp' e '.mat']));
assignin('base', 'navdata', S.navdata);

in = Simulink.SimulationInput(mdl);
in = in.setModelParameter('StopTime', num2str(S.navdata.time(end)));
[~, out] = evalc('sim(in)');          % evalc: o bloco de replay imprime "t = ..." a cada passo

d.t        = out.acc_log.time;
N          = numel(d.t);
d.acc      = fix2d(out.acc_log,  N);
d.gyro     = fix2d(out.gyro_log, N);
d.phi_ob   = squeeze(out.roll_onboard.signals.values);
d.theta_ob = squeeze(out.pitch_onboard.signals.values);
d.phi_m    = atan2d(-d.acc(:,2), -d.acc(:,3));
d.theta_m  = atan2d( d.acc(:,1), sqrt(d.acc(:,2).^2 + d.acc(:,3).^2));

xhat = fix2d(out.state_estimate, N);
[d.x, d.phi, d.theta, d.b, d.sb] = kf5_outputs(xhat, out.P_estimate.signals.values);
end

function X = fix2d(s, N)
X = squeeze(s.signals.values);
if size(X,1) ~= N, X = X.'; end
end
