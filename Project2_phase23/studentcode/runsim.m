close all;
clear all;
clc;
addpath(genpath('./'));

%% Plan path
disp('Planning ...');

scenario = 2;
phase = 3;

if scenario == 1
    map = loadMap('maps/map1.txt', 0.25, 2.0, 0.3);
    start = [5  -2 3];
    stop  = [5  18 3];
elseif scenario == 2
    map = loadMap('maps/map2.txt', 0.1, 5.0, 0.2);
    start = [0.5  1 4.75];
    stop  = [3.8  1 0.25];
elseif scenario == 3
    map = loadMap('maps/map3.txt', 0.1, 1.0, 0.2);
    start = [1  2 5.5];
    stop  = [19.5 3 2];
end

path = dijkstra(map, start, stop, true);

h = figure(1);
path = trim_path(path);
plot_path(map, path, h);

%% Run trajectory
if phase == 2
    plot_trajectory(map, path);
elseif phase == 3
    test_trajectory(start, stop, map, path, true);
end
