function plotLynx(q)

global lynx pennkeys

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variables you can adjust.  I guess you can adjust whatever you want, but
% we recommend sticking to these =)
jointSize = 20;
frameSize = 100;
linkWidth = 2;
shadowWidth = 2;
axisWidth = 2;
gripperLineWidth = 3;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if lynx.pwm_check 
    q_g = lynxPWM(lynx, q);
end

if lynx.has_gravity
    tau = gravity_to_torque(q, lynx);
    q_g = lynxAddGravity(lynx, q, tau)
end

calculateFK = str2func(['calculateFK_',pennkeys]);
[jointPositions,T0e] = calculateFK(q);
[jointPositions_g, T0e_g] = calculateFK(q); 

if(lynx.firstFrame) %We need to create the plots
    clf; %Clear any previous lynx plot
    subplot(1,2,1); 
    title('Original Simulation');
    xlabel('X (mm)')
    ylabel('Y (mm)')
    zlabel('Z (mm)')
    set(gca,'xtick',-1000:100:1000, 'ytick',-1000:100:1000,'ztick',-1000:100:1000)
    grid on
    view(80,20)

    axis equal vis3d
    xlim([-200 400]);
    ylim([-400 400]);
    zlim([-200 500]);

    lynx.hLinks = line(jointPositions(:,1),jointPositions(:,2),jointPositions(:,3),'color',[0.2,0.2,0.2],'LineWidth',linkWidth);
    hold on;

    if(lynx.showShadow)
        lynx.hShadow = line(jointPositions(:,1),jointPositions(:,2),zeros(size(jointPositions,1),1),'color',[0.5,0.5,0.5],'LineWidth',shadowWidth);
    end

    if(lynx.showJoints)
        lynx.hFrameOrigins = scatter3(jointPositions(:,1),jointPositions(:,2),jointPositions(:,3),jointSize,'MarkerEdgeColor','none','MarkerFaceColor',[0,0,0]);
    end

    if(lynx.showFrame)
        EExAxis = T0e(1:3,1,end)';
        EEyAxis = T0e(1:3,2,end)';
        EEzAxis = T0e(1:3,3,end)';
        EExAxis = frameSize*EExAxis;
        EEyAxis = frameSize*EEyAxis;
        EEzAxis = frameSize*EEzAxis;
        lynx.hEExAxis = line([T0e(1,4),T0e(1,4,end)+EExAxis(1)],[T0e(2,4,end),T0e(2,4,end)+EExAxis(2)],[T0e(3,4,end),T0e(3,4,end)+EExAxis(3)],'Color','r','LineWidth',axisWidth);
        lynx.hEEyAxis = line([T0e(1,4),T0e(1,4,end)+EEyAxis(1)],[T0e(2,4,end),T0e(2,4,end)+EEyAxis(2)],[T0e(3,4,end),T0e(3,4,end)+EEyAxis(3)],'Color','g','LineWidth',axisWidth);
        lynx.hEEzAxis = line([T0e(1,4),T0e(1,4,end)+EEzAxis(1)],[T0e(2,4,end),T0e(2,4,end)+EEzAxis(2)],[T0e(3,4,end),T0e(3,4,end)+EEzAxis(3)],'Color','b','LineWidth',axisWidth);
    end

    if(lynx.showGripper)
        g = q(6) + 8; %Gripper width
        l = 30; % Gripper length
        gripperPoints(1,:) = [g/2,0,l];
        gripperPoints(2,:) = [g/2,0,0];
        gripperPoints(3,:) = [-g/2,0,0];
        gripperPoints(4,:) = [-g/2,0,l];
        gripperPoints = jointPositions(6,:)' + T0e(1:3,1:3) * [gripperPoints]';
        gripperPoints = gripperPoints';
        lynx.hGripper = line(gripperPoints(:,1),gripperPoints(:,2),gripperPoints(:,3),'color',[0,0,0],'LineWidth',gripperLineWidth);
    end
    
    
    % second subplot to show gravity:
    subplot(1,2,2); 
    title('Simulation with Gravitational Effects');
    xlabel('X (mm)')
    ylabel('Y (mm)')
    zlabel('Z (mm)')
    set(gca,'xtick',-1000:100:1000, 'ytick',-1000:100:1000,'ztick',-1000:100:1000)
    grid on
    view(80,20)

    axis equal vis3d
    xlim([-200 400]);
    ylim([-400 400]);
    zlim([-200 500]);

    lynx.hLinks = line(jointPositions_g(:,1),jointPositions_g(:,2),jointPositions_g(:,3),'color',[0.2,0.2,0.2],'LineWidth',linkWidth);
    hold on;

    if(lynx.showShadow)
        lynx.hShadow = line(jointPositions_g(:,1),jointPositions_g(:,2),zeros(size(jointPositions_g,1),1),'color',[0.5,0.5,0.5],'LineWidth',shadowWidth);
    end

    if(lynx.showJoints)
        lynx.hFrameOrigins = scatter3(jointPositions_g(:,1),jointPositions_g(:,2),jointPositions_g(:,3),jointSize,'MarkerEdgeColor','none','MarkerFaceColor',[0,0,0]);
    end

    if(lynx.showFrame)
        EExAxis = T0e_g(1:3,1,end)';
        EEyAxis = T0e_g(1:3,2,end)';
        EEzAxis = T0e_g(1:3,3,end)';
        EExAxis = frameSize*EExAxis;
        EEyAxis = frameSize*EEyAxis;
        EEzAxis = frameSize*EEzAxis;
        lynx.hEExAxis = line([T0e_g(1,4),T0e_g(1,4,end)+EExAxis(1)],[T0e_g(2,4,end),T0e_g(2,4,end)+EExAxis(2)],[T0e_g(3,4,end),T0e_g(3,4,end)+EExAxis(3)],'Color','r','LineWidth',axisWidth);
        lynx.hEEyAxis = line([T0e_g(1,4),T0e_g(1,4,end)+EEyAxis(1)],[T0e_g(2,4,end),T0e_g(2,4,end)+EEyAxis(2)],[T0e_g(3,4,end),T0e_g(3,4,end)+EEyAxis(3)],'Color','g','LineWidth',axisWidth);
        lynx.hEEzAxis = line([T0e_g(1,4),T0e_g(1,4,end)+EEzAxis(1)],[T0e_g(2,4,end),T0e_g(2,4,end)+EEzAxis(2)],[T0e_g(3,4,end),T0e_g(3,4,end)+EEzAxis(3)],'Color','b','LineWidth',axisWidth);
    end

    if(lynx.showGripper)
        g = q_g(6) + 8; %Gripper width
        l = 30; % Gripper length
        gripperPoints(1,:) = [g/2,0,l];
        gripperPoints(2,:) = [g/2,0,0];
        gripperPoints(3,:) = [-g/2,0,0];
        gripperPoints(4,:) = [-g/2,0,l];
        gripperPoints = jointPositions_g(6,:)' + T0e_g(1:3,1:3) * [gripperPoints]';
        gripperPoints = gripperPoints';
        lynx.hGripper = line(gripperPoints(:,1),gripperPoints(:,2),gripperPoints(:,3),'color',[0,0,0],'LineWidth',gripperLineWidth);
    end

    % Let the code know to use 'set' from now on, instead of recreating the
    % plots themselves.  This allows the plotting to happen much more
    % quickly, and is really worth understanding for future work!
    lynx.firstFrame = false;

else %Plots are already created
    subplot(1,2,1);
    set(lynx.hLinks,'xdata',jointPositions(:,1),'ydata',jointPositions(:,2),'zdata',jointPositions(:,3));
    subplot(1,2,2);
    set(lynx.hLinks,'xdata',jointPositions_g(:,1),'ydata',jointPositions_g(:,2),'zdata',jointPositions_g(:,3));

    
    if(lynx.showShadow)
        subplot(1,2,1);
        set(lynx.hShadow,'xdata',jointPositions(:,1),'ydata',jointPositions(:,2));
        subplot(1,2,2);
        set(lynx.hShadow,'xdata',jointPositions_g(:,1),'ydata',jointPositions_g(:,2));
    end

    if(lynx.showJoints)
        subplot(1,2,1); 
        set(lynx.hFrameOrigins,'xdata',jointPositions(:,1),'ydata',jointPositions(:,2),'zdata',jointPositions(:,3));
        subplot(1,2,2); 
        set(lynx.hFrameOrigins,'xdata',jointPositions_g(:,1),'ydata',jointPositions_g(:,2),'zdata',jointPositions_g(:,3));
    end

    if(lynx.showFrame)
        subplot(1,2,1);
        EExAxis = T0e(1:3,1,end)';
        EEyAxis = T0e(1:3,2,end)';
        EEzAxis = T0e(1:3,3,end)';
        EExAxis = frameSize*EExAxis;
        EEyAxis = frameSize*EEyAxis;
        EEzAxis = frameSize*EEzAxis;
        set(lynx.hEExAxis,'xdata',[T0e(1,4,end),T0e(1,4,end)+EExAxis(1)],'ydata',[T0e(2,4,end),T0e(2,4,end)+EExAxis(2)],'zdata',[T0e(3,4,end),T0e(3,4,end)+EExAxis(3)]);
        set(lynx.hEEyAxis,'xdata',[T0e(1,4,end),T0e(1,4,end)+EEyAxis(1)],'ydata',[T0e(2,4,end),T0e(2,4,end)+EEyAxis(2)],'zdata',[T0e(3,4,end),T0e(3,4,end)+EEyAxis(3)]);
        set(lynx.hEEzAxis,'xdata',[T0e(1,4,end),T0e(1,4,end)+EEzAxis(1)],'ydata',[T0e(2,4,end),T0e(2,4,end)+EEzAxis(2)],'zdata',[T0e(3,4,end),T0e(3,4,end)+EEzAxis(3)]);
    
        subplot(1,2,2);
        EExAxis = T0e_g(1:3,1,end)';
        EEyAxis = T0e_g(1:3,2,end)';
        EEzAxis = T0e_g(1:3,3,end)';
        EExAxis = frameSize*EExAxis;
        EEyAxis = frameSize*EEyAxis;
        EEzAxis = frameSize*EEzAxis;
        set(lynx.hEExAxis,'xdata',[T0e_g(1,4,end),T0e_g(1,4,end)+EExAxis(1)],'ydata',[T0e_g(2,4,end),T0e_g(2,4,end)+EExAxis(2)],'zdata',[T0e_g(3,4,end),T0e_g(3,4,end)+EExAxis(3)]);
        set(lynx.hEEyAxis,'xdata',[T0e_g(1,4,end),T0e_g(1,4,end)+EEyAxis(1)],'ydata',[T0e_g(2,4,end),T0e_g(2,4,end)+EEyAxis(2)],'zdata',[T0e_g(3,4,end),T0e_g(3,4,end)+EEyAxis(3)]);
        set(lynx.hEEzAxis,'xdata',[T0e_g(1,4,end),T0e_g(1,4,end)+EEzAxis(1)],'ydata',[T0e_g(2,4,end),T0e_g(2,4,end)+EEzAxis(2)],'zdata',[T0e_g(3,4,end),T0e_g(3,4,end)+EEzAxis(3)]);
    end
    
    % assuming that gripper is not shown for the most part 
    if(lynx.showGripper)
        g = q(6)+ 8; %Gripper width plus a bit for the physical size
        l = 30; % Gripper length
        gripperPoints(1,:) = [g/2,0,l];
        gripperPoints(2,:) = [g/2,0,0];
        gripperPoints(3,:) = [-g/2,0,0];
        gripperPoints(4,:) = [-g/2,0,l];
        gripperPoints = jointPositions(6,:)' + T0e(1:3,1:3) * [gripperPoints]';
        gripperPoints = gripperPoints';
        set(lynx.hGripper,'xdata',gripperPoints(:,1),'ydata',gripperPoints(:,2),'zdata',gripperPoints(:,3));
    end

end

drawnow

end
