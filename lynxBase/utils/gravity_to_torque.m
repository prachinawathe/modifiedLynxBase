function tau = gravity_to_torque(q, lynx)
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
robot = lynx.robot; 

% Weights of the individual components in grams
link_weights = robot.link_weights;
joint_weights = robot.joint_masses;

% Calculate the weights seen by each joint for a given Jacobian calculation
w_joint1 = link_weights(1) + joint_weights(2);
w_joint2 = link_weights(2) + joint_weights(3);
w_joint3 = link_weights(3) + joint_weights(4);

% Joint 4 carries the whole wrist
w_joint4 = link_weights(4) + joint_weights(5) + link_weights(5) + joint_weights(6) + link_weights(6);

% Make this a vector for ease of summation
w = [w_joint1 w_joint2 w_joint3 w_joint4];
tau = zeros(1,6);
z0 = [0 0 1]';

% There are 4 joints to consider
for i = 1:4

    % Get the linear Jacobian to the center of mass for this joint
    Jv = calcJacobian_gravity(q, i+1, robot);

    % Calculate the torque due to this link and add to total
    % There is no multiplication by g const because we have weights, not
    % masses
    tau_g = Jv'*w(i)*z0;
    tau(1:size(tau_g)) = tau(1:size(tau_g)) + tau_g';
end

end