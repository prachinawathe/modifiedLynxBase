function q = lynxInertia(lynx, q)

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
if isempty(lynx.dt)
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

[m,n] = size(joint_pos);

r_sq = joint_pos(2:m,1).^2 + joint_pos(2:m,2).^2;
I(1) = I(1) + robot.joint_masses(1) * 15^2 + ...
    robot.joint_masses(2:m) * r_sq; 


% convert link_com to radius 

link_r = link_com(:,1) .^ 2 + link_com(:,2) .^ 2; 
I(1) = I(1) + robot.link_weights * link_r; 
I(1) = I(1) + robot.link_weights(1) * 15^2; 

for i=2:4
    x = joint_pos(i:m,1) - joint_pos(i,1);
    z = joint_pos(i:m,3) - joint_pos(i,3);
    linkx = link_com(i:m, 1) - joint_pos(i,1);
    linkz = link_com(i:m, 3) - joint_pos(i,3); 
    r_sq = x.^2 + z.^2; 
    linksq = linkx .^ 2 + linkz.^2; 
    I(i) = I(i) + robot.joint_masses(i:m) * r_sq; 
    I(i) = I(i) + robot.link_weights(i:m) * linksq; 
end

% this servo is radial, assuming 5mm radius of end eff. servo
% and 10mm radius of end effector itself

I(5) = I(5) + robot.joint_masses(6) * 5^2; 
I(5) = I(5) + robot.link_weights(6) * 10^2;




% modify q here with J and F

end