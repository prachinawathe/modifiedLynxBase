%% Setup

close all
addpath('utils')
addpath('maps')

profile on

%% Simulation Parameters
%
% Define any additional parameters you may need here. Add the parameters to
% the robot and map structures to pass them to astar or rrt.
%
robot = load('robot.mat');

robot.mrgn = .1; 

start = [0,0,0,0,0,0];
% target of [111.957896202101;-307.859940416223;315.460411913033]
goal = [pi/3,-pi/6,pi/3,0,0,0];

map = loadmap('emptyMap.txt');

% Realistic sim parameters
plot_realistic = 1;
gravity_comp = 0;

%% Run the simulation

%% Run the simulation

% Turn the map into a C-space map
tic
cmap = getCMap(map,robot);
toc
% Solve the path problem
tic
[path, num] = astar(cmap, start, goal);
toc
% OR
%[path] = rrt(cmap,start,goal);

profile off

%% Plot the output

[pos_ideal, pos_real] = plotLynxPath(map,path, 2, plot_realistic, gravity_comp);

% Plot the different end effector trajectories
if plot_realistic
   figure()
   plotmap(map);
   xlabel('X (mm)')
   ylabel('Y (mm)')
   zlabel('Z (mm)')
   set(gca,'xtick',-1000:100:1000, 'ytick',-1000:100:1000,'ztick',-1000:100:1000);
   grid on
   view(80,20)

   axis equal vis3d
   xlim([-200 400]);
   ylim([-400 400]);
   zlim([-200 500]);
   plot3(pos_ideal(1,:), pos_ideal(2,:), pos_ideal(3,:));
   hold on
   plot3(pos_real(1,:), pos_real(2,:), pos_real(3,:));
   title('End Effector Paths');
   plot3(pos_ideal(1,1), pos_ideal(2,1), pos_ideal(3,1), ...
       'o','Color','k','MarkerSize',5,'MarkerFaceColor','#FFFFFF');
   plot3(pos_ideal(1,end), pos_ideal(2,end), pos_ideal(3,end), ...
       'o','Color','k','MarkerSize',5,'MarkerFaceColor','#000000');
   hold off
   if gravity_comp
       legend('Idealistic simulation', ...
              'Simulation with gravity compensation',...
              'Start', 'Goal', 'Location', 'northeast');  
   else
       legend('Idealistic simulation',...
               'Simulation with gravity effects',...
               'Start', 'Goal', 'Location', 'northeast');
   end
   
   % Add a little note about how far the realistic sim ends up at the end
   pos_diff = pos_real(:,end) - pos_ideal(:,end);
   str = sprintf("Distance from ideal\n at goal:\n X = %.2f mm\n Y = %.2f mm\n Z = %.2f mm\n", ...
              pos_diff(1,1), pos_diff(2,1), pos_diff(3,1));
   annotation('textbox',[.75 .4 .2 .25],'String',str,'EdgeColor','k')

end
%profile viewer
