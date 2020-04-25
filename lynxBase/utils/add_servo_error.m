function q_new = add_servo_error(q, robot)
% add_servo_error - Add in slight error to the input configuration
%       This function adds some small angular error to each joint based on
%       the dead space for each joint's servo and how far each servo moves 
%       per us added to the PWM signal.
% INPUTS:
%   q - A 1x6 vector containing the robot's desired configuration
%   robot - A struct containing information about the robot's physical 
%           properties
% OUTPUTS:
%   q_new - The configuration after some servo error is added
%

% Convert from degrees per us to radians per us
deg_per_us = robot.deg_per_us;
min_diff_rad = deg_per_us * 2 * pi / 360;

% All 6 servo spec sheets had the same deadspace
deadspace_us = robot.deadspace_us;

% Get the deadspace in radians and then create the upper and lower bounds
% for the possible configuration
deadspace_rad = deadspace_us*min_diff_rad;
lower_bound = q - (deadspace_rad/2);
upper_bound = q + (deadspace_rad/2);

% Make q_new some random number within the deadspace
q_new = (upper_bound - lower_bound).*rand(1,6) + lower_bound;

end

