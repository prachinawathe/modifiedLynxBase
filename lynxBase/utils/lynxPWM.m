function q = lynxPWM(lynx, q)
%% lynxPWM - Models servo errors and adds them to the lynx joint configuration.
%   The two types of effects modeled here are servo deadband and servo
%   position error coming from position measurement error. The deadband
%   limits the smallest possible step size for each joint. The position
%   error is modeled here as a function of only potentiometer manufacturing
%   tolerances and is assumed to be constant for a given robot simulation.
% INPUTS:
%   lynx - Struct containing lynx information
%   q - 1x6 vector containing desired joint configuration
% OUTPUTS:
%   q - Updated joint configurations with deadspace and position error
%       added.

persistent q_prev
persistent q_err_factor
persistent deadspace_rad

if isempty(lynx.robot)
    disp('Error: must start PWM checks with hardware set to Sim');
    return; 
end

% This runs during sim initialization
if isempty(q_prev)
    
    % Load servo parameters from the robot
    robot = lynx.robot;
    deg_per_us = robot.deg_per_us;

    % All 6 servo spec sheets had the same deadspace
    deadspace_us = robot.deadspace_us;

    % Convert smallest movement size to radians
    min_diff_rad = deg_per_us * 2 * pi / 360;

    % Get the deadspace in radians and then create the upper and lower bounds
    % for the possible configuration
    deadspace_rad = deadspace_us*min_diff_rad;
    
    % Position error = position reading error. This is modeled using assumed
    % potentiometer tolerance.
    pos_error = robot.servo_pos_error;
    
    % Set the error factor as some constant value based on potentiometer
    % tolerance. It can vary from sim to sim, like from robot to robot, but
    % shouldn't change for a given robot.
    q_err_factor = ((1 + pos_error) - (1-pos_error)).*rand(1,6) + (1-pos_error);
    q_prev = [0,0,0,0,0,0];
    return;
end

% If the desired change in configuration is too small (i.e. inside deadspace),
% don't move that joint.
q_diff = abs(q-q_prev);
mask = q_diff < deadspace_rad;

% Change q values based on position sensing error
q = q.*q_err_factor;

% Restore joints which shouldn't have moved to their previous values
q(mask) = q_prev(mask); 
q_prev = q;

end 

