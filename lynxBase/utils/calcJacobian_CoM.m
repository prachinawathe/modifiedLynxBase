function J = calcJacobian_CoM(q, joint, robot)
% CALCJACOBIAN_CoM Calculate the Jacobian of a particular joint of the 
%   robot in a given configuration while considering the center of mass of
%   each link.
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   joint - scalar in [2,6] representing which joint we care about
%   robot - a struct of robot parameters
%
% OUTPUTS:
%   J - 6 x (joint-1) matrix representing the Jacobian
%

%%

J = [];

if nargin < 3
    disp('Error: not enough arguments to calcJacobian');
    return
elseif joint <= 1 || joint > 6
     disp('Error in calcJacobian: Please enter a joint between 2 and 6');
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

% REPLACE ME I AM FAKE NUMBERS
m_link1 = 1;
m_link2 = 1;
m_link3 = 1;
m_link4 = 1;
m_link5 = 1;
m_servo1 = 1;
m_servo2 = 1;
m_servo3 = 1;
m_servo4 = 1;
m_servo5 = 1;
m_gripper = 1;

% Calculate each link's center of mass
CoM_1 = calc_link_CoM(m_link1, m_servo2, d1, 0);
CoM_2 = calc_link_CoM(m_link2, m_servo3, a2, 0);
CoM_3 = calc_link_CoM(m_link3, m_servo4, a3, 0);
CoM_4 = calc_link_CoM(m_link4, m_servo5, d4, 0);
CoM_5 = calc_link_CoM(m_link5, m_gripper, d5, lg);

% Create homogeneous transforms from the DH parameters
A1 = homogeneous_transform(0, d1, -pi/2, theta1);
A2 = homogeneous_transform(a2, 0, 0, theta2 - pi/2);
A3 = homogeneous_transform(a3, 0, 0, theta3 + pi/2);
A4 = homogeneous_transform(0, 0, -pi/2, theta4 - pi/2);
A5 = homogeneous_transform(0, d5 + lg, 0, theta5);

% Calculate homogeneous transforms based on each link's center of mass
B1 = homogeneous_transform(0, CoM_1, -pi/2, theta1);
B2 = homogeneous_transform(CoM_2, 0, 0, theta2 - pi/2);
B3 = homogeneous_transform(CoM_3, 0, 0, theta3 + pi/2);
B4 = homogeneous_transform(CoM_4, 0, 0, theta4); % Joint 4 is weird
B5 = homogeneous_transform(0, CoM_5, 0, theta5);

% Create transformation matrices
T_01 = A1;
T_02 = A1*A2;
T_03 = A1*A2*A3;
T_04 = A1*A2*A3*A4;
T_05 = A1*A2*A3*A4*A5;

% Create transformation matrices from each link's CoM as seen from frame 0
TC_01 = B1;
TC_02 = T_01*B2;
TC_03 = T_02*B3;
TC_04 = T_03*B4;
TC_05 = T_04*B5;

% Get the origin of each joint according to DH
o0 = [0 0 0]';
o1 = T_01(1:3,4);
o2 = T_02(1:3,4);
o3 = T_03(1:3,4);
o4 = T_04(1:3,4);
o5 = T_05(1:3,4);

% Get the position of each center of mass with reference to frame 0
c1 = TC_01(1:3,4);
c2 = TC_02(1:3,4);
c3 = TC_03(1:3,4);
c4 = TC_04(1:3,4);
c5 = TC_05(1:3,4);

% Define Z axes with reference to frame 0
z0 = [0 0 1]';
z1 = T_01(1:3,1:3)*z0;
z2 = T_02(1:3,1:3)*z0;
z3 = T_03(1:3,1:3)*z0;
z4 = T_04(1:3,1:3)*z0;
z5 = T_05(1:3,1:3)*z0;

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
    J = [Jv_1; 
        Jw];
  return
end

% JOINT 3
if (joint == 3)
    Jv_1 = cross(z0, c2-o0);
    Jv_2 = cross(z1, c2-o1);
    Jw = [z0 T_01(1:3,3)];
    J = [Jv_1 Jv_2; 
        Jw];
  return
end

% JOINT 4
if (joint == 4)
    Jv_1 = cross(z0, c3-o0);
    Jv_2 = cross(z1, c3-o1);
    Jv_3 = cross(z2, c3-o2);
    Jw = [z0 T_01(1:3,3) T_02(1:3,3)];
    J = [Jv_1 Jv_2 Jv_3; 
        Jw];
  return
end

% JOINT 5
if (joint == 5)
    Jv_1 = cross(z0, c4-o0);
    Jv_2 = cross(z1, c4-o1);
    Jv_3 = cross(z2, c4-o2);
    Jv_4 = cross(z3, c4-o3);
    Jw = [z0 T_01(1:3,3) T_02(1:3,3) T_03(1:3,3)];
    J = [Jv_1 Jv_2 Jv_3 Jv_4; 
        Jw];
  return
end

% JOINT 6
if (joint == 6)
    Jv_1 = cross(z0, c5-o0);
    Jv_2 = cross(z1, c5-o1);
    Jv_3 = cross(z2, c5-o2);
    Jv_4 = cross(z3, c5-o3);
    Jv_5 = cross(z4, c5-o4);
    Jw = [z0 T_01(1:3,3) T_02(1:3,3) T_03(1:3,3) T_04(1:3,3)];
    J = [Jv_1 Jv_2 Jv_3 Jv_4 Jv_5; 
        Jw];
  return
end

end