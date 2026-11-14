function P = lab2_paths()
%LAB2_PATHS folders used by the scripts (root, lib, data, figs)
%   everything needed is inside group17_lab2: lib/ (ARBlocks library and DevKit model files) and data/

P.root = fileparts(mfilename('fullpath'));

% ARBlocks library and the DevKit files it needs (setupARModel, getWaypoints, decode_32bit, ss*.mat)
P.lib = fullfile(P.root, 'lib');
if ~exist(fullfile(P.lib, 'ARBlocks.slx'), 'file')
    error('lab2_paths:lib', 'ARBlocks.slx not found in %s', P.lib);
end
addpath(P.lib);

% data expA..expE.mat
P.data = fullfile(P.root, 'data');

% figures saved by the plot scripts
P.figs = fullfile(P.root, 'figs');
if ~exist(P.figs, 'dir'), mkdir(P.figs); end
end
