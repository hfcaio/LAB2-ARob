%% setup for the simulation of sections 4.1 and 4.4 (ARDroneHoverSim_4_1 and ARDroneHoverSim_4_4)
% gyro = derivative of roll/pitch + noise (+ bias in _4_4), inclinometer = angle + noise
% the filter is chosen afterwards with setup_4_1 or setup_4_4
% does not use the DevKit setupHoverSim.m because it runs clear all / bdclose all

here = fileparts(mfilename('fullpath'));
P = lab2_paths();

% drone model (same as setupHoverSim.m)
FMS.Ts    = 0.065;
timeDelay = FMS.Ts*4;
setupARModel;
waypoints = getWaypoints();
simDT     = 0.005;
Tsim      = 60;

% manoeuvre: step in x (pitch) and then in y (roll)
tStepX = 10;  xRefStep = 1;     % m
tStepY = 30;  yRefStep = 1;     % m

% noise covariances
sensor_cov;

% signals added to the measurements (From Workspace blocks)
bias_true_val = [-0.03 0.05];   % [b_p b_q] rad/s, artificial gyro bias
t  = (0:simDT:Tsim).';
N  = numel(t);
rng(17);                        % repeatable noise
noise_gyro = mkSignal(t, randn(N,2) .* sqrt([R_p R_q]));        % rad/s, zero mean
noise_incl = mkSignal(t, randn(N,2) .* sqrt([Q_phi Q_theta]));  % rad, zero mean
bias_gyro  = mkSignal(t, repmat(bias_true_val, N, 1));           % only used in _4_4

% signal in Structure With Time format
function s = mkSignal(t, v)
s.time = t;
s.signals.values = v;
s.signals.dimensions = size(v, 2);
end
