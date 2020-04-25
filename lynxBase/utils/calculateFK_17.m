function [jointPositions,T0e] = calculateFK_17(q)
% CALCULATEFK_GROUPNO - Please rename this function using your group # in
%   both the function header and the file name.
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Your code here
jointPositions=zeros(6,3);
% T0e = eye(4,4);

d1 = 3*25.4; 

% From robot.mat
d4 = 34;
d5 = 52.75;
lg = 28.575;

% From measurements - converting 3 in to cm
%d_elbow_wrist = 3*25.4;

A01 = homogeneous_transform(0, d1, pi/2, q(1));
A12 = homogeneous_transform(5.75*25.4, 0, 0, -1*q(2) + pi/2);
A23 = homogeneous_transform(7.375*25.4, 0, 0, -1*q(3) - pi/2);
A34 = homogeneous_transform(0, 0, pi/2, -1*q(4) + pi/2);

% Distance from the elbow to the center of the wrist is the d value here
A45 = homogeneous_transform(0, d5, 0, q(5));

% Isolate the actual location of joint 5
% No rotations because this distance and the rest of d_elbow_wrist are on
% the same plane
A54 = homogeneous_transform(0, d4 - d5, 0, 0);

% Frame 5 is center of wrist, e is end of the end effector
A5e = homogeneous_transform(0, lg, 0, 0);

T0e = A01; 
jointPositions(2, 1:3) = T0e(1:3, 4)';
T0e = T0e * A12;
jointPositions(3, 1:3) = T0e(1:3, 4)';
T0e = T0e * A23;
jointPositions(4, 1:3) = T0e(1:3, 4)';
T0e = T0e * A34;
T0e = T0e * A45;
jointPositions(6, 1:3) = T0e(1:3, 4)';

% Go back to fill in joint 5
T04 = T0e * A54;
jointPositions(5, 1:3) = T04(1:3, 4)';

% Include the length of the end effector
T0e = T0e * A5e;

%T0e = T0e * A56;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end