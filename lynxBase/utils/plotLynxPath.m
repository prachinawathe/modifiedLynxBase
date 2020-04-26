function plotLynxPath(map, path, totalT)
% PLOTLYNXPATH Plots the lynx following a set of waypoints produced by a
%   mapping algorithm such as Astar or RRT
%
% INPUTS
%   map - a struct containing the workspace map
%   path - a set of Nx6 configurations of the lynx
%   totalT - the total time for the plotting (s)
%
% OUTPUTS
%   N/A
%
% AUTHOR
%   Gedaliah Knizhnik - knizhnik@seas.upenn.edu

global lynx

if nargin < 2
    totalT = 10;
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

if startLynx
    lynxStart()
    pause(1.5)
    lynxServo(path(1,1),path(1,2),path(1,3),path(1,4),path(1,5),path(1,6));
end

% If hardware is not being used, plot the map on the simulation
if ~lynx.hardware_on
    plotmap(map)
end

pause(1.5)

% Go through the steps
for ii=1:size(path,1)-1
    morePath = (1-t)*path(ii,:) + t*path(ii+1,:);
    
    for jj = 1:size(morePath,1)
        lynxServo(morePath(jj,:));
        pause(totalT/n/size(path,1))
    end

end

end