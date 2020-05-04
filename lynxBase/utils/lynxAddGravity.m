function q = lynxAddGravity(lynx, q, tau)

% lynxInertia calculate the inertia of the robot wrt 
% every single joint 
%
% INPUTS:
%   q     - 1 x 6 vector of joint inputs [q1,q2,q3,q4,q5,q6]
%   lynx
%
% OUTPUTS:
%   I - 1 x 6 vector representing the inertia 
%

if isempty(lynx.robot)
    disp('Error: must start inertia checks with hardware set to Sim');
    return; 
end


robot = lynx.robot; 

[joint_pos, T0e] = calculateFK_17(q);

I = zeros(1,6);

% calculate link CoMs 
end_pts = joint_pos;
end_pts = cat(1, end_pts, T0e(1:3, 4)'); 

link_com = movsum(end_pts, 2) / 2;
link_com = link_com(2:end, :); 

% a better generalization of inertia would be 
% to assume that the links are rods whose perpendicular 
% components behave like a point pass and whose 
% parallel components behave like a rod being rotated
% around its endpoint 

% for each joint, calculate displacement
% for end effector, assume radius of 15 mm? 

% joint 1 -- not sure how to vectorize this 
% joint_axis = [0 0 1]';
% get joint pos from fk

[M,n] = size(joint_pos);

r_sq = joint_pos(2:M,1).^2 + joint_pos(2:M,2).^2;
I(1) = I(1) + robot.joint_masses(1) * 15^2 + ...
    robot.joint_masses(2:M) * r_sq; 


% convert link_com to radius 
R = sqrt(r_sq);
R = R(2:4)';
l = robot.link_lengths;
l(4) = sum(l(4:6));
l = l(2:4);
m = robot.link_weights(2:4);
m(3) = sum(robot.link_weights(4:6));

I(1) = I(1) + sum(m .* (R + l .* cos(q(2:4))).^2 + ...
    m .* R .^2 + m .* R .* l .* cos(q(2:4)) + m / 3 .* l.^2 .* cos(q(2:4)).^2);

for i=2:4
    x = joint_pos(i:M,1) - joint_pos(i,1);
    z = joint_pos(i:M,3) - joint_pos(i,3);
    linkx = link_com(i:M, 1) - joint_pos(i,1);
    linkz = link_com(i:M, 3) - joint_pos(i,3); 
    r_sq = x.^2 + z.^2; 
    linksq = sqrt(linkx .^ 2 + linkz.^2);
    
    l = robot.link_lengths;
    l(4) = sum(l(4:6));
    R = sqrt(r_sq(1:M-i-1))';
    l = l(i:4);
    m = robot.link_weights;
    m(4) = sum(robot.link_weights(4:6));
    m = m(i:4);
    theta = q(i:4) - sum(q(2:i));
    
    I(i) = I(i) + robot.joint_masses(i:M) * r_sq; 
%     I(i) = I(i) + robot.link_weights(i:m) * linksq; 

    I(i) = I(i) + sum(m .* (R + l .* cos(theta)).^2 + ...
    m .* R .^2 + m .* R .* l .* cos(theta) + m / 3 .* l.^2 .* cos(theta).^2);
end

% this servo is radial, assuming 5mm radius of end eff. servo
% and 10mm radius of end effector itself

I(5) = I(5) + robot.joint_masses(6) * 5^2; 
I(5) = I(5) + robot.link_weights(6) * 10^2;

% modify q here with J and F
alpha = tau(1:5) ./ I(1:5);
q(1:5) = q(1:5) - .5 * alpha .* lynx.dt ^ 2;


end