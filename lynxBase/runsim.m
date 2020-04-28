%% Setup

close all
addpath('utils')
addpath('maps')

profile on

%% Simulation Parameters
%
% Define any additional parameters you may need here. Add the parameters to
% the robot and map structures to pass them to astar or rrt.
%
robot = load('robot.mat')% robot

robot.mrgn = .1; 

start = [0,0,0,0,0,0];
% target of [111.957896202101;-307.859940416223;315.460411913033]
goal = [-1.2220,0.6570,-1.1157,-0.0650,0,0];

map = loadmap('map2.txt');


%% Run the simulation

%% Run the simulation

% Turn the map into a C-space map
tic
cmap = getCMap(map,robot);
toc
% Solve the path problem
tic
[path, num] = astar(cmap, start, goal);
toc
% OR
%[path] = rrt(cmap,start,goal);

profile off

%% Plot the output

plotLynxPath(map,path,10);

profile viewer
