function [isCollided] = isRobotCollided(q, map, robot)
% ISROBOTCOLLIDED Detect if a configuration of the Lynx is in collision
%   with any obstacles on the map.
%
% INPUTS:
%   q   - a 1x6 configuration of the robot
%   map - a map strucutre containing axis-aligned-boundary-box obstacles
%   robot - a structure containing the robot dimensions and joint limits
%
% OUTPUTS:
%   isCollided - a boolean flag: 1 if the robot is in collision, 0 if not.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Starts Here             %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[joint_pos, T0e] = calculateFK_17(q);
szobstacle = size(map.obstacles);
obstacles = map.obstacles; 

% disp(T0e(1:3,4)');

% If there are no obstacles, return immediately
if szobstacle(1) == 0
    isCollided = false;
    return
end

% change the dimensions of the obstacles to account for robot volume
obstacles(:,1:3) = obstacles(:, 1:3) - 10; 
obstacles(:,4:6) = obstacles(:, 4:6) + 10; 

% x1 = obstacles(1);
% x2 = obstacles(4);
% y1 = obstacles(2);
% y2 = obstacles(5);
% z1 = obstacles(3);
% z2 = obstacles(6);
% if x1 <= T0e(1,4) && T0e(1,4) <= x2
%     if y1 <= T0e(2,4) && T0e(2,4) <= y2
%         if z1 <= T0e(3,4) && T0e(3,4) <= z2
%             disp('collision');
%         end
%     end
% end
% check for self collision
pos = T0e(1:3, 4);
if pos(1) < 10 && pos(1) > -10
    if pos(2) < 15 && pos(2) > -15
        if pos(3) < 15 && pos(3) > -15
            isCollided = true;
            return
        end
    end
end

if szobstacle(1) == 1
    isCollided = detectCollision(joint_pos(2,:), joint_pos(3,:), obstacles) ...
                |detectCollision(joint_pos(3,:), joint_pos(6,:), obstacles);
%     if isCollided
%         disp('return isCollided');
%     end
    return
end


for i=1:szobstacle(1)
    isCollided = detectCollision(joint_pos(2,:), joint_pos(3,:), obstacles(i,:)) ...
                | detectCollision(joint_pos(3,:), joint_pos(6,:), obstacles(i,:));
    if isCollided
        break; 
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                  Algortihm Ends Here               %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end
