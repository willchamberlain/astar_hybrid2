addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'

%%

floorplan_x = 7 % 7m forward
floorplan_y = 4 % 5m left
floorplan_scale = 0.1  % 10 cells/metre

    % orientation_space = 2*pi 
    % orientation_space_divisions = 8
    % orientation_scale = orientation_space/orientation_space_divisions   %  45 degree increments 
    % floorplan_space = zeros(  ceil(floorplan_y*floorplan_scale)  ,  ceil(floorplan_x*floorplan_scale)  ) ;

cost_to_divert = floorplan_space ;
cost_to_observe = floorplan_space ;

%%  1)  cost_to_observe as measurement model 
%{
    Put the Robot_Target somewhere in the world
    Work out the field/cost to each cell
    Heatmap 
%}
figure; hold on; grid on; xlabel('distance (0.1m)'); ylabel('observability');
%    Put the Robot_Target somewhere in the world
robot_target_posn = [ ...
    round( randi( 10*floorplan_y) ), ...
    round( randi( 10*floorplan_x ) )  ];
% robot_target_posn = [   round(10*floorplan_y/2), round(10*floorplan_x/2) ]  ;
costmap_obs_dist = zeros(10*floorplan_y,10*floorplan_x)  ;
max_dist = sqrt( (10*floorplan_y)^2 + (10*floorplan_x)^2 ) ;
% CALCULATE COST
for yy_ = 1.0 : 1 : 10*floorplan_y
    for xx_ = 1.0 : 1 : 10*floorplan_x
        dist_cell_to_target = sqrt(   [ yy_- robot_target_posn(1) ]^2  +   [ xx_- robot_target_posn(2) ]^2   )  ;        
        costmap_obs_dist(yy_, xx_) =  (0.00275*exp(dist_cell_to_target*0.185*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
        if costmap_obs_dist(yy_, xx_) > 1 
            costmap_obs_dist(yy_, xx_) = 1  ;
        elseif costmap_obs_dist(yy_, xx_) < 0  
            costmap_obs_dist(yy_, xx_) = 0  ;
        end
    end
end

% VISUALISE COST
figure
subplot(1,2 , 1);  colormap hot
surf( costmap_obs_dist )  
subplot(1,2 , 2);  colormap hot
surf( 1-costmap_obs_dist )  

%%
costmap_dist_to_path_follower = zeros(10*floorplan_y,10*floorplan_x)  ;
size(costmap_dist_to_path_follower)  ;

num_iterations = 60 ;
robot_posn = [5,5] ;
f1h=figure_named('costmap_dist_to_path_follower'); hold on; grid on; xlabel('x'); ylabel('y');
f2h=figure_named('costmap_obs_dist '); hold on; grid on; xlabel('x'); ylabel('y');
f3h=figure_named('costmap_obs_dist + costmap_dist_to_path_follower '); hold on; grid on; xlabel('x'); ylabel('y');

main_task_start_point=[30,10]  ;
main_task_end_point=[30,60]  ;
path_follower_step=(main_task_end_point-main_task_start_point) / num_iterations  ;
path_follower_posn = main_task_start_point  ;

robot_start_posn = [ 60, 10 ]  ;
robot_end_posn = [ 10, 20 ]  ;
robot_step = ( robot_end_posn - robot_start_posn ) / num_iterations  ;
robot_target_posn = robot_start_posn  ;

max_dist = sqrt( (10*floorplan_y)^2 + (10*floorplan_x)^2 ) ;
max_cost = max_dist;

for t = 1:num_iterations
    figure(f1h)
    axes = gca;
    clf;
    grid on; hold on; 
    path_follower_posn = path_follower_posn + path_follower_step  ;
    costmap_dist_to_path_follower = zeros(10*floorplan_y,10*floorplan_x)  ;
    
    robot_target_posn = robot_target_posn + robot_step  ;
    
    for yy_ = 1.0 : 1 : 10*floorplan_y
        for xx_ = 1.0 : 1 : 10*floorplan_x
            dist_cell_to_target = sqrt(   [ yy_- robot_target_posn(1) ]^2  +   [ xx_- robot_target_posn(2) ]^2   )  ;        
            costmap_obs_dist(yy_, xx_) =  (0.00275*exp(dist_cell_to_target*0.185*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
            if costmap_obs_dist(yy_, xx_) > 1 
                costmap_obs_dist(yy_, xx_) = 1  ;
            elseif costmap_obs_dist(yy_, xx_) < 0  
                costmap_obs_dist(yy_, xx_) = 0  ;
            end
        end
    end
    
    for yy_ = 1.0 : 1 : 10*floorplan_y
        for xx_ = 1.0 : 1 : 10*floorplan_x
            costmap_dist_to_path_follower(yy_,xx_) ...
                = sqrt(  (yy_ - path_follower_posn(1))^2  + (xx_ - path_follower_posn(2))^2)    /  max_dist;            
        end
    end
    colormap hot  ;
    surf( costmap_dist_to_path_follower )  ;
    plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  max(max(costmap_dist_to_path_follower)) ] , 'cx'  , 'LineWidth',5)
    view(3);
    
    figure(f2h)  ;
    clf;
    grid on; hold on; 
    colormap hot  ;
    costmap_total = costmap_obs_dist ;
    plot3(robot_target_posn(2),robot_target_posn(1),  max(max(costmap_total)) , 'mx', 'LineWidth',5)   
    surf( costmap_total )  ;
    
    figure(f3h)  ;
    clf;
    grid on; hold on; 
    colormap hot  ;
    costmap_total = costmap_obs_dist + costmap_dist_to_path_follower ;
    surf( costmap_total )  ;
    
%       figure
%     plot3(robot_target_posn(1),robot_target_posn(2),  max(max(costmap_total)) , 'cx', 'LineWidth',5)   
    plot3(robot_target_posn(2),robot_target_posn(1),  max(max(costmap_total)) , 'mx', 'LineWidth',5)   
    
    %     axis equal  ;
    daspect(  [  1 1 0.1  ]  );
    xlim( [0 70] )
    ylim( [0 40] )
    zlim( [-0.1 1.7] )
    axes = gca;
        %     axes.View( [-15.600000000000046 , 29.200000000000024]')    
        %     view  ( 37, 30 )
    view(3) ;
    pause  ;
end

%%
[ path__ , f_score, g_score , came_from, open_set, closed_set ]  ...
    = path_planning__astar(...
    costmap_total, ...
    main_task_start_point, ...
    main_task_end_point)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] ...
    = path_planning__reconstruct_path(came_from, main_task_end_point)    ;
plot3(main_task_end_point(2), main_task_end_point(1), max(max(costmap_total)) , 'cx', 'LineWidth',5) 
plot3( 10, 30, 1.5, 'cx', 'LineWidth',5) 
factor_ = 10;
path_planning__draw_came_from(came_from, strcat(  'bbbooobbb'    ,': ',sprintf(' factor %7.f',factor_)))    ;  
path_planning__draw_path( total_path_goal_to_start__ )    ;

