close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');
map_num = 0;
map = load_map(['maps/map' num2str(map_num) '.txt'], 0.1, 1.0, 0.25);

% Determine start and stop
if map_num == 0
    %start = {[2.0 2.0 2.0]}; %map0
    % Random start between -5 and 5 in X, Y, Z
    start = {10 * (rand(1, 3) - 0.5)};
    stop = {[0.0 0.0 0.0]};
elseif map_num == 1
    start = {[2.0 -2.0 1.0]}; %map1
    stop = {[8.0 18.0 2.5]};
elseif map_num == 2
    start = {[0.1 5.0 1.0]}; %map2
    stop = {[5.0 10.0 3.0]};
elseif map_num == 3
    start = {[2.0 3 5.0]}; %map3
    stop = {[18.0 4.0 5.0]};
else
    error('Map number not recognized');
end

nquad = length(start);

%% Additional init script
init_script;

%% Run trajectory
trajectory = test_trajectory(start, stop, map, path, true); % with visualization
