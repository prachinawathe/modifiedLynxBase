function [path, num_expanded] = astar(map, start, goal)
% ASTAR Find the shortest path from start to goal.
%   PATH = ASTAR(map, start, goal) returns an mx6 matrix, where each row
%   consists of the configuration of the Lynx at a point on the path. The
%   first row is start and the last row is goal. If no path is found, PATH
%   is a 0x6 matrix. Consecutive points in PATH should not be farther apart
%   than neighboring voxels in the map (e.g. if 5 consecutive points in
%   PATH are co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = ASTAR(map, start, goal) finds the path using euclidean
%   distance to goal as a heuristic.
%
%   [PATH, NUM_EXPANDED] = ASTAR(...) returns the path as well as
%   the number of nodes that were expanded while performing the search.
%
% INPUTS:
%   map     - the map object to plan in (cartesian map)
%   start   - 1x6 vector of the starting configuration
%   goal:   - 1x6 vector of the goal configuration


%% Prep Code

path = [];
num_expanded = 0;

% Optional but highly recommended
% cmap = getCmap(map);
% map is already given in C-space
cmap = map; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%{ 
pseudocode:
openset (unexplored configs)
fscore (best score to reach each config from start)
gscore (score at each config)
came_from (used to reconstruct path, 4D matrix)
visited (configurations where the lowest scores have already been found)

while openset is not empty:
    current = node in openset with lowest fscore
    if current is goal, reconstruct path to goal
    openset.remove(current)
    for each neighbor of current: (+/- cmap.res for theta_{1,2,3}) 
%     6 total neighbors
        temp_gscore = gscore(current) + cmap.rec
        if temp_gscore < gscore[neighbor] and neighbor is valid config
            gscore[neighbor] = temp_gscore
            fscore[neighbor] = gscore[neighbor] + dist(neighbor, goal)
            came_from[neighbor] = current
        if neighbor not in openset:
            openset.add(neighbor)
failure


gscore calculation: distance from 
%}
lower = cmap.bound_th123;
goalidx = int8((goal(1:3)-lower(1:3))/.05 + 1);

if cmap.occgrid(goalidx(1), goalidx(2), goalidx(3)) == 1
    path = zeros(0,6);
    return
end

gscore = inf(size(cmap.occgrid));
fscore = inf(size(cmap.occgrid));
openset = zeros(size(cmap.occgrid));
visited = zeros(size(cmap.occgrid));
% convert start to indices

i = int8((start(1) - lower(1)) / .05 + 1);
j = int8((start(2) - lower(2)) / .05 + 1); 
k = int8((start(3) - lower(3)) / .05 + 1); 


num_expanded = 0; 

% check if starting position is valid 
[t1, t2, t3] = size(cmap.occgrid);
if 1 > i || i > t1 || j < 1 || j > t2 || k < 1 || k > t3
    path = zeros(0,6);
    disp('Starting position is invalid');
    return 
end
if 1 > goalidx(1) || goalidx(1) > t1 || goalidx(2) < 1 ||...
        goalidx(2) > t2 || goalidx(3) < 1 || goalidx(3) > t3
    path = zeros(0,6);
    disp('Goal position is invalid');
    return 
end
came_from = zeros(t1, t2, t3, 3);
gscore(i, j, k) = 0;
fscore(i, j, k) = dist([i,j,k], goalidx);

openset(i,j,k) = 1; % all calcs in index form
while max(openset, [], 'all') ~= 0
%     get minimum fscore element from openset
    minf = inf;
    for i=1:t1
        for j=1:t2
            for k=1:t3
                if fscore(i,j,k) < minf && openset(i,j,k) == 1 && ...
                        visited(i,j,k) == 0
                    minf = fscore(i,j,k);
                    minidx = [i,j,k];
                end
            end
        end
    end
    current = minidx;
    %disp(current);
    num_expanded = num_expanded + 1; 
    openset(minidx(1), minidx(2), minidx(3)) = 0;
    if current == goalidx
        %disp('found goal');
        path = reconstruct_path(came_from, current, lower);
        return % this should be the path reconstruction
    end
    
%     for each neighbor of current
    neighbors = zeros(6,3);
    neighbors(:,1) = current(1);
    neighbors(:,2) = current(2);
    neighbors(:,3) = current(3);
    neighbors(1,1) = neighbors(1,1) + 1;
    neighbors(2,1) = neighbors(2,1) - 1;
    neighbors(3,2) = neighbors(3,2) + 1;
    neighbors(4,2) = neighbors(4,2) - 1;
    neighbors(5,3) = neighbors(5,3) + 1;
    neighbors(6,3) = neighbors(6,3) - 1;
    for i=1:6
        if neighbors(i,1) < 1 || neighbors(i,1) > t1 || ...
                neighbors(i,2) < 1 || neighbors(i,2) > t2 || ...
                neighbors(i,3) < 1 || neighbors(i,3) > t3
            continue
        end
        temp_gscore = gscore(current(1), current(2), current(3)) + 1;
        if temp_gscore < gscore(neighbors(i,1), neighbors(i,2), neighbors(i,3)) && ...
                cmap.occgrid(neighbors(i,1), neighbors(i,2), neighbors(i,3)) == 0
            gscore(neighbors(i,1), neighbors(i,2), neighbors(i,3)) = temp_gscore;
            fscore(neighbors(i,1), neighbors(i,2), neighbors(i,3)) = ...
                gscore(neighbors(i,1), neighbors(i,2), neighbors(i,3)) ...
                + dist(neighbors(i,:), goalidx);
            came_from(neighbors(i,1), neighbors(i,2), neighbors(i,3),:) = ...
                current;
        end
        if openset(neighbors(i,1), neighbors(i,2), neighbors(i,3)) == 0 && ...
                cmap.occgrid(neighbors(i,1), neighbors(i,2), neighbors(i,3)) == 0
            openset(neighbors(i,1), neighbors(i,2), neighbors(i,3)) = 1;
        end
    end
    visited(minidx(1), minidx(2), minidx(3)) = 1; 
end

path = zeros(0,6);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algorithm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


end

% before a configuration is valid, check that each of the three
% links don't intersect with the prisms 
% (get their location by calling calc_FK function)

% write a new IK function to give thetas 1-3 
% safe to ignore thetas 4, 5 because any position reachable with 
% an angle on t4 is also reachable with just t1, t2, t3 and t4 at 0

% heuristic 

function d = dist(start, goal)
    d = abs(start(1)-goal(1)) + abs(start(2)-goal(2)) + abs(start(3)-goal(3));
end

function path = reconstruct_path(came_from, current, lower)
    path = idx2q(current, lower);
    while came_from(current(1), current(2), current(3),:) ~= 0
        current = came_from(current(1), current(2), current(3),:);
        path = cat(1, idx2q(current, lower), path);
    end
end

function q = idx2q(current, lower)
    q = zeros(1,6);
    q(1) = (current(1)-1)*.05 + lower(1);
    q(2) = (current(2)-1)*.05 + lower(2);
    q(3) = (current(3)-1)*.05 + lower(3);
end