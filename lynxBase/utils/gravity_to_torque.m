function tau = gravity_to_torque(q, robot)
%gravity_to_torque  Calculates how the force of gravity on the robot
%                   translates to a torque
%   Calculates the linear Jacobian at the center of mass of the first 
%   4 joints (Joint 5 and the gripper are not subject to gravity), then 
%   multipies the transpose of that Jacobian by the force of gravity acting
%   on each joint. The sum of all J^T*m*g*z_0 is the the effect gravity has
%   on all of the joint torques.
%   INPUTS:
%       q - A 1x6 vector containing the robot joint configuration
%       robot - Stored robot variables
%   OUTPUTS:
%       tau - A 1x6 vector containing the torque changes on the robot 
%               joints due to gravity
%%

% Define the gravity accleration constant
G_CONST = -9800;

% FAKE VALUES CHANGE ME
% Masses of the individual components in grams
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

% Calculate the masses seen by each joint for a given Jacobian calculation
m_joint1 = m_link1 + m_servo2;
m_joint2 = m_link2 + m_servo3;
m_joint3 = m_link3 + m_servo4;

% Joint 4 carries the whole wrist
m_joint4 = m_link4 + m_servo5 + m_link5 + m_gripper;

% Make this a vector for ease of summation
masses = [m_joint1 m_joint2 m_joint3 m_joint4];
tau = zeros(1,6);
z0 = [0 0 1]';

% There are 4 joints to consider
for i = 1:4
    
    % Convert the mass to kilograms
    m = masses;
    
    % Get the linear Jacobian to the center of mass for this joint
    Jv = calcJacobian_gravity(q, 1i+1, robot);
    
    % Calculate the force due to gravity on this joint's load
    Fg = m*G_CONST*z0;
    
    % Calculate the torque due to this link and add to total
    tau_g = Jv'*Fg;
    tau(1:size(tau_g)) = tau(1:size(tau_g)) + tau_g';
end

end

