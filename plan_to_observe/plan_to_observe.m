addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'

%%

floorplan_w = 7 % 5m
floorplan_x = floorplan_w
floorplan_l = 4 % 5m
floorplan_y = floorplan_l
floorplan_scale = 0.1  % 10 cells/metre

orientation_space = 2*pi 
orientation_space_divisions = 8
orientation_scale = orientation_space/orientation_space_divisions   %  45 degree increments 

floorplan_space = zeros(  ceil(floorplan_l*floorplan_scale)  ,  ceil(floorplan_w*floorplan_scale)  ) ;


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
    round( randi( 10*floorplan_l) ), ...
    round( randi( 10*floorplan_w ) )  ];
% robot_target_posn = [   round(10*floorplan_l/2), round(10*floorplan_w/2) ]  ;
costmap_obs_dist = zeros(10*floorplan_l,10*floorplan_w)  ;
max_dist = sqrt( (10*floorplan_l)^2 + (10*floorplan_w)^2 ) ;
% CALCULATE COST
for length_ = 1.0 : 1 : 10*floorplan_l
    for width_ = 1.0 : 1 : 10*floorplan_w
        dist_cell_to_target = sqrt(   [ length_- robot_target_posn(1) ]^2  +   [ width_- robot_target_posn(2) ]^2   )  ;        
        costmap_obs_dist(length_, width_) =  (0.00275*exp(dist_cell_to_target*0.185*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
        if costmap_obs_dist(length_, width_) > 1 
            costmap_obs_dist(length_, width_) = 1  ;
        elseif costmap_obs_dist(length_, width_) < 0  
            costmap_obs_dist(length_, width_) = 0  ;
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
costmap_dist_to_path_follower = zeros(10*floorplan_l,10*floorplan_w)  ;
size(costmap_dist_to_path_follower)  ;

num_iterations = 60 ;
robot_posn = [5,5] ;
f1h=figure_named('costmap_dist_to_path_follower'); hold on; grid on; xlabel('x'); ylabel('y');
f2h=figure_named('costmap_obs_dist + costmap_dist_to_path_follower '); hold on; grid on; xlabel('x'); ylabel('y');

main_task_start_point=[30,10]  ;
main_task_end_point=[30,60]  ;
path_follower_step=(main_task_end_point-main_task_start_point) / num_iterations  ;
path_follower_posn = main_task_start_point  ;

max_dist = sqrt( (10*floorplan_l)^2 + (10*floorplan_w)^2 ) ;
max_cost = max_dist;

for t = 1:num_iterations
    figure(f1h)
    axes = gca;
    clf;
    grid on; hold on; 
    path_follower_posn = path_follower_posn + path_follower_step  ;
    costmap_dist_to_path_follower = zeros(10*floorplan_l,10*floorplan_w)  ;
    
    
    for length_ = 1.0 : 1 : 10*floorplan_l
        for width_ = 1.0 : 1 : 10*floorplan_w
            costmap_dist_to_path_follower(length_,width_) ...
                = sqrt(  (length_ - path_follower_posn(1))^2  + (width_ - path_follower_posn(2))^2)    /  max_dist;            
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
    costmap_total = costmap_obs_dist + costmap_dist_to_path_follower ;
    surf( costmap_total )  ;
    
    plot3(robot_target_posn(1),robot_target_posn(2),  max(max(costmap_total)) , 'cx', 'LineWidth',5)   
    plot3(robot_target_posn(2),robot_target_posn(1),  max(max(costmap_total)) , 'cx', 'LineWidth',5)   
    
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



