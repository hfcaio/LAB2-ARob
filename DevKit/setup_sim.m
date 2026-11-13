%% setup file for LAB2 — simulação (ARDroneHoverSim_4_1 sem bias, ARDroneHoverSim_4_4 com bias)
%  O modelo deriva roll/pitch para simular o gyro, soma ruído gaussiano (e bias no _4_4),
%  e soma ruído ao ângulo para simular o inclinómetro. O bloco kalman_filter é o mesmo do
%  replay e usa A, B, C, KALMAN_GAIN: correr setup_4_1 ou setup_4_4 depois deste ficheiro.
%  (não usa setupHoverSim.m porque esse faz clear all / bdclose all)

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));

% --- parâmetros do modelo do drone (iguais aos de setupHoverSim.m)
FMS.Ts    = 0.065;
timeDelay = FMS.Ts*4;
setupARModel;
waypoints = getWaypoints();
simDT     = 0.005;
Tsim      = 60;

% --- manobra: degrau em x (pitch) e depois em y (roll)
tStepX = 10;  xRefStep = 1;     % m
tStepY = 30;  yRefStep = 1;     % m

% --- covariâncias do ruído (o filtro é escolhido à parte: setup_4_1 ou setup_4_4)
sensor_cov;

% --- sinais somados às medidas (From Workspace)
bias_true_val = [-0.03 0.05];   % [b_p b_q] rad/s, bias artificial do gyro
t  = (0:simDT:Tsim).';
N  = numel(t);
rng(17);                        % ruído repetível
noise_gyro = mkSignal(t, randn(N,2) .* sqrt([R_p R_q]));        % rad/s, média zero
noise_incl = mkSignal(t, randn(N,2) .* sqrt([Q_phi Q_theta]));  % rad,   média zero
bias_gyro  = mkSignal(t, repmat(bias_true_val, N, 1));           % só usado no _4_4

function s = mkSignal(t, v)
s.time = t;
s.signals.values = v;
s.signals.dimensions = size(v, 2);
end
