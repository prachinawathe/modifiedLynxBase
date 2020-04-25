function J = calcJacobian_gravity(q, joint, robot)
% CALCJACOBIAN_CoM Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration specifically to calculate the
%   gravitational forces on the first 4 joints
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [2,5] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 3 x (joint-1) matrix representing the linear velocity Jacobian
%

%%

J = [];

if nargin < 3
    disp('Error: not enough arguments to calcJacobian');
    return
elseif joint <= 1 || joint > 5
     disp('Error in calcJacobian: Please enter a joint between 2 and 5');
    return
end

% Take out q values so it's slightly easier to read and type
theta1 = q(1);
theta2 = q(2);
theta3 = q(3);
theta4 = q(4);
theta5 = q(5);

% Define rigid link lengths
d1 = robot.d1;
a2 = robot.a2;
a3 = robot.a3;
d4 = robot.d4;
d5 = robot.d5;
lg = robot.lg;
link_weights = robot.link_weights;
joint_weights = robot.joint_weights;

% Calculate each link's center of mass
CoM_1 = calc_link_CoM(link_weights(1), joint_weights(2), d1);
CoM_2 = calc_link_CoM(link_weights(2), joint_weights(3), a2);
CoM_3 = calc_link_CoM(link_weights(3), joint_weights(4), a3);

% The center of mass for the 4th joint requires considering the various
% elements of the wrist, at least in the gripper config
CoM_4 = calc_wrist_CoM(link_weights(4), joint_weights(5), link_weights(5), joint_weights(6), link_weights(6),...
                                d4, 0, (d5 - d4), lg);


% Create homogeneous transforms from the DH parameters
A1 = homogeneous_transform(0, d1, -pi/2, theta1);
A2 = homogeneous_transform(a2, 0, 0, theta2 - pi/2);
A3 = homogeneous_transform(a3, 0, 0, theta3 + pi/2);
A4 = homogeneous_transform(0, 0, -pi/2, theta4 - pi/2);

% Calculate homogeneous transforms based on each link's center of mass
B1 = homogeneous_transform(0, CoM_1, -pi/2, theta1);
B2 = homogeneous_transform(CoM_2, 0, 0, theta2 - pi/2);
B3 = homogeneous_transform(CoM_3, 0, 0, theta3 + pi/2);
B4 = homogeneous_transform(CoM_4, 0, 0, theta4); % Joint 4 is weird

% Create transformation matrices
T_01 = A1;
T_02 = A1*A2;
T_03 = A1*A2*A3;
T_04 = A1*A2*A3*A4;

% Create transformation matrices from each link's CoM as seen from frame 0
TC_01 = B1;
TC_02 = T_01*B2;
TC_03 = T_02*B3;
TC_04 = T_03*B4;

% Get the origin of each joint according to DH
o0 = [0 0 0]';
o1 = T_01(1:3,4);
o2 = T_02(1:3,4);
o3 = T_03(1:3,4);

% Get the position of each center of mass with reference to frame 0
c1 = TC_01(1:3,4);
c2 = TC_02(1:3,4);
c3 = TC_03(1:3,4);
c4 = TC_04(1:3,4);

% Define Z axes with reference to frame 0
z0 = [0 0 1]';
z1 = T_01(1:3,1:3)*z0;
z2 = T_02(1:3,1:3)*z0;
z3 = T_03(1:3,1:3)*z0;
z4 = T_04(1:3,1:3)*z0;

%%%
% Calculate the Jacobian for the input joint. Which origin is considered
% the "end" depends on which joint we are solving for. Input joint = end.
%%%

% Size sanity check
J = zeros(6,joint-1);

% JOINT 2
if (joint == 2)
    Jv_1 = cross(z0, c1-o0);
    Jw = z0;
    J = Jv_1; 
  return
end

% JOINT 3
if (joint == 3)
    Jv_1 = cross(z0, c2-o0);
    Jv_2 = cross(z1, c2-o1);
    J = [Jv_1 Jv_2];
  return
end

% JOINT 4
if (joint == 4)
    Jv_1 = cross(z0, c3-o0);
    Jv_2 = cross(z1, c3-o1);
    Jv_3 = cross(z2, c3-o2);
    J = [Jv_1 Jv_2 Jv_3];
  return
end

% JOINT 5
if (joint == 5)
    Jv_1 = cross(z0, c4-o0);
    Jv_2 = cross(z1, c4-o1);
    Jv_3 = cross(z2, c4-o2);
    Jv_4 = cross(z3, c4-o3);
    J = [Jv_1 Jv_2 Jv_3 Jv_4];
  return
end

end