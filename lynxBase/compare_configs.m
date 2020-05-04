% This script is intended to provide an easy way to compare static
% positions for the idealized simulation and one with some effects added in
% It plots both Lynx robot simulations side by side and measures the
% difference in end effector postions

%% Setup
close all
addpath('utils')

%% Desired q configuration here
q_pose = [1 -1 -1 0 0 0];

%%
% Start simuation, move to desired pose, 
% and disply ideal sim alongside more realistic sim
lynxStart('Hardware', 'Sim');
[q_ideal, q_real] = lynxServo(q_pose);

% Find the distance in X,Y, and Z in the two end effector positions
[~,T0e_ideal] = calculateFK_17(q_ideal);
[~,T0e_real] = calculateFK_17(q_real);
pos_ideal = T0e_ideal(1:3,4);
pos_real = T0e_real(1:3,4);
pos_diff = (pos_real - pos_ideal);
str = sprintf("End effector is this far from ideal in mm:\n X = %.2f \n Y = %.2f \n Z = %.2f\n", ...
              pos_diff(1,1), pos_diff(2,1), pos_diff(3,1));
disp(str);