function [q, q_g] = lynxServoRealSim(lynx, th1, th2, th3, th4, th5, grip)
% lynxServoRealSim shows two simulated outputs. One of them is the original
% simulation with no modeling of real world effects, and the other is a
% simulation which has had the force of gravity and servo error added

% INPUTS:
    % th1...th5 : Joint variables for the six DOF Lynx arm (rad)
    % grip = gripper width (mm)

    % Ensure that the Lynx has been initialized
    if(~evalin('base','exist(''lynx'',''var'')'))
        error('lynxMove:lynxInitialization','Lynx has not been initialised. Run lynxStart first');
    end

    % Check inputs (whether q is 6 entries or single vector)
    if nargin == 7 && length(th1) == 1
        q = [th1 th2 th3 th4 th5 grip];
    elseif nargin == 2 && length(th1) == 6 && numel(th1) == 6
        q = th1;
        % Ensure that q is a row vector
        if size(q, 1) > 1
            q = q';
        end
    else
        error('lynxMove:argumentCheck','Input must be lynx followed by 6 scalars or a 6 element vector representing desired joint angles.')
    end

    if any(isnan(q))
        error('lynxMove:nanInput',['Input contains a value of NaN. Input was: [' num2str(q) ']']);
    end

    % Add servo error
    q_g = lynxPWM(lynx, q);
    
    % Calculate the torques that gravity imposes and then use the robot's
    % moment of inertia to convert back to joint configurations
    tau = gravity_to_torque(q_g, lynx);
    
    % This takes longer than it needs to, but is the theoreticlly correct
    % way to compensate for gravity. It would be more meaningful if this
    % were a real robot and not fake gravity being fake compensated.
    if lynx.gravity_comp
        tau = tau - gravity_to_torque(q_g,lynx);
    end
    q_g = lynxAddGravity(lynx, q_g, tau);
    % Changes the Lynx config to that specified by input and plots it there
    plot_2_Lynxes(q, q_g);


end

