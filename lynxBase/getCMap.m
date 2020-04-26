function [cmap] = getCMap(map, robot)
% GETCMAP calculates an occupancy grid in 3D C space based on a map object
%   defining obstacles in cartesian space. Effectively, this function turns
%   a cartesian space map into a configuration space map.
%
% INPUTS
%   map  - a map struct containing a 1x6 boundary field defining the
%           boundaries of the space and an Nx6 obstacles array where each
%           row defines an axis aligned bounding box (x1,y1,z1,x2,y2,z2).
%           This struct is produced by the loadMap function.
%   robot - a struct containing the dimensions and joint limits of the
%           Lynx. This is stored in robot.mat.
%
% * You should make sure that the above inputs contain the following
%   fields:
%
%   map.res_th123 - a 3x1 vector defining the resolution for theta1,
%                   theta2, and theta3.
%   robot.mrgn    - the safety margin for contact between the Lynx and any
%                   obstacles.
%
% OUTPUTS
%   cmap - an equivalent map structure for c space, containing an occupancy
%           grid that can be used for graph searches.
%           cmap should have the following fields:
%
%       cmap.occgrid     - a 3D matrix containing 1 for occupied voxels and
%                           0 for free voxels. Note that the indexing is
%                           th1,th2,th3.
%       cmap.bound_th123 - 1x6 containing lower bounds for th1, th2, th3
%                           and then upper bounds for th1, th2, th3.
%       cmap.res_th123   - 1x3 containing the resolution for th1, th2, th3
%       cmap.res_th12    - 1x2 containing the resolution for th1, th2
%       cmap.res_th3     - 1x1 containing the resolution for th3
%       cmap.mrgn        - 1x1 containing the margin of safety for the Lynx
%                           (in mm).

%% Write your code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

cmap = struct;
cmap.mrgn = robot.mrgn;

% iterate through all t1, t2, t3 options
lower = robot.lowerLim(1:3);
upper = robot.upperLim(1:3);

occgrid = zeros(int8((upper(1)-lower(1))/.05 + 1), ... 
                int8((upper(2)-lower(2))/.05 + 1), ...
                int8((upper(3)-lower(3))/.05 + 1));
            
% modify the map to account for robot geometry 

for i = lower(1):.05:upper(1)
    for j=lower(2):.05:upper(2)
        for k=lower(3):.05:upper(3)
            % conversion to occgrid indices
            idx = int8((i - lower(1)) / .05 + 1);
            jdx = int8((j - lower(2)) / .05 + 1); 
            kdx = int8((k - lower(3)) / .05 + 1); 
            % may have to cast to int if we run into issues
            % check for intersection with boxes in map
            if isRobotCollided([i,j,k,0,0,0], map, robot)
                occgrid(idx, jdx, kdx) = 1;
%                 disp('found collision');
            else 
                occgrid(idx, jdx, kdx) = 0; 
            end
        end
    end
    
end
    cmap.occgrid = occgrid;
    cmap.bound_th123 = [lower(1), lower(2), lower(3), ...
                        upper(1), upper(2), upper(3)];
    % cmap res's all need to be changed
    cmap.res_th123 = [.05, .05, .05]; 
    cmap.res_th12 = [.05, .05];
    cmap.res_th3 = [.05];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
