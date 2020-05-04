function [pos_ideal, pos_real] = plotLynxPath(map, path, totalT, plot_realistic, gravity_comp)
% PLOTLYNXPATH Plots the lynx following a set of waypoints produced by a
%   mapping algorithm such as Astar or RRT
%
% INPUTS
%   map - a struct containing the workspace map
%   path - a set of Nx6 configurations of the lynx
%   totalT - the total time for the plotting (s)
%   plot_realistic - Plot a second, more realistic simulation
%   gravity_comp - More realistic simulation has gravity compensation
%                  turned on
%
% OUTPUTS
%  pos_ideal - a set of Nx3 positions of the idealisitc end effector
%              positions. This is an empty set if not running a 
%              realistic sim.
%  pos_real - a set of Nx3 positions of the realistic end effector
%             positions. This is an empty set if not running a 
%             realistic sim.
%
% AUTHOR
%   Gedaliah Knizhnik - knizhnik@seas.upenn.edu
%   Soe small modifications for running multiple paths at once by project
%   team 10, spring 2020.

global lynx

if nargin < 2
    totalT = 10;
    plot_realisitic = 0;
    gravity_comp = 0;
elseif nargin < 3
    plot_realisitic = 0;
    gravity_comp = 0; 
elseif nargin < 4
    gravity_comp = 0;     
end

n = 20;
t = linspace(0,1,n)';

% If lynx has not been started, start it now
if isempty(lynx) 
    startLynx = 1;
elseif ~ishandle(lynx.hLinks)
    startLynx = 1;
else
    startLynx = 0;
end

if startLynx && plot_realistic
    if gravity_comp
        lynxStart('Realistic','on','Gravity_comp','on');
    else
        lynxStart('Realistic','on','Gravity_comp','off');        
    end
    pause(1.5)
    lynxServo(path(1,1),path(1,2),path(1,3),path(1,4),path(1,5),path(1,6));
elseif startLynx
    lynxStart()
    pause(1.5)
    lynxServo(path(1,1),path(1,2),path(1,3),path(1,4),path(1,5),path(1,6));
end

% If hardware is not being used, plot the map on the simulation
if ~lynx.hardware_on
    plotmap(map)
end

if plot_realistic
    pos_real = zeros(3,size(path,1));
    pos_ideal = zeros(3,size(path,1));
    cnt = 1;
else
    pos_real = [];
    pos_ideal = [];
end

pause(1.5)

% Go through the steps
for ii=1:2:size(path,1)-1
    morePath = (1-t)*path(ii,:) + t*path(ii+1,:);
    
    for jj = 1:size(morePath,1)
        if plot_realistic
            [q_ideal, q_real] = lynxServo(morePath(jj,:));
            [~,T0e_ideal] = calculateFK_17(q_ideal);
            [~,T0e_real] = calculateFK_17(q_real);
            pos_ideal(:,cnt) = T0e_ideal(1:3,4);
            pos_real(:,cnt) = T0e_real(1:3,4);
            cnt = cnt + 1;
        else
            lynxServo(morePath(jj,:));
        end
        
        pause(totalT/n/size(path,1))

    end

end

end