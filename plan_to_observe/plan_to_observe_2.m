addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'

%%

floorplan_x = 11 % 7m forward
floorplan_y = 9 % 5m left
floorplan_extent_m = [floorplan_x;floorplan_y];
floorplan_scale = 0.1  % 10 cells/metre
floorplan_extent_cells = floorplan_extent_m.*(1/floorplan_scale)

    % orientation_space = 2*pi 
    % orientation_space_divisions = 8
    % orientation_scale = orientation_space/orientation_space_divisions   %  45 degree increments 
floorplan_space = zeros(  ceil(floorplan_x*floorplan_scale) , ceil(floorplan_y*floorplan_scale)  ) ;
cost_to_divert = floorplan_space ;
cost_to_observe = floorplan_space ;


%%  1)  cost_to_observe as measurement model 
%{
    Put the Robot_Target somewhere in the world
    Work out the field/cost to each cell
    Heatmap 
%}
figure; hold on; grid on; xlabel('distance (0.1m)'); ylabel('observability');
draw_axes_direct(eye(3),[floorplan_extent_cells(1)/2;floorplan_extent_cells(2)/2;0],'',1)
%    Put the Robot_Target somewhere in the world
robot_target_posn = [ ...
    round( randi( (1/floorplan_scale)*floorplan_x) ); ...
    round( randi( (1/floorplan_scale)*floorplan_y ) )  ];
robot_target_posn = round(floorplan_extent_cells/2)   ;
costmap_obs_dist = ones(floorplan_extent_cells(1),floorplan_extent_cells(2))  ;
max_dist = sqrt( ((1/floorplan_scale)*floorplan_x)^2 + ((1/floorplan_scale)*floorplan_y)^2 ) ;
% CALCULATE COST
for yy_ = 1.0 : 1 : floorplan_extent_cells(2)
    for xx_ = 1.0 : 1 : floorplan_extent_cells(1)
        dist_cell_to_target = [ xx_ ; yy_ ] - robot_target_posn  ;
        dist_cell_to_target = norm_2(dist_cell_to_target,1)  ;
                            % dist_cell_to_target = sqrt(   [ - robot_target_posn(1) ]^2  +   [ xx_- robot_target_posn(2) ]^2   )  ;        
        costmap_obs_dist(xx_ , yy_) =  (0.00275*exp((dist_cell_to_target)*0.215*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
        costmap_obs_dist(xx_ , yy_) =  (0.00275*exp((dist_cell_to_target)*0.1215*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
        if costmap_obs_dist(xx_ , yy_) > 1 
            costmap_obs_dist(xx_ , yy_) = 1  ;
        elseif costmap_obs_dist(xx_ , yy_) < 0  
            costmap_obs_dist(xx_ , yy_) = 0  ;
        end
    end
end

% VISUALISE COST
figure_named('Observation benefit/gain/measurement model vs distance'); 
axes=subplot(1,2 , 1);  colormap hot; title('Observation "cost"/measurement model') ;   hold on; grid on; xlabel('x (0.1m)'); ylabel('y (0.1m)'); zlabel('cost')
surf( costmap_obs_dist )  ;  view(3)  ;  
draw_axes_direct(eye(3),[floorplan_extent_cells(1)/2;floorplan_extent_cells(2)/2;0],'',1)
axes.DataAspectRatio=[1 1 0.02]


axes=subplot(1,2 , 2);  colormap hot; title('Observation benefit/gain/measurement model falls with distance') ;  hold on; grid on; xlabel('x (0.1m)'); ylabel('y (0.1m)'); zlabel('cost')
surf( 1-costmap_obs_dist )  ;  view(3)  ;
draw_axes_direct(eye(3),[floorplan_extent_cells(1)/2;floorplan_extent_cells(2)/2;0],'',1)
axes.DataAspectRatio=[1 1 0.02]


%%
costmap_dist_to_path_follower = zeros(floorplan_extent_cells')  ;
size(costmap_dist_to_path_follower)  

num_iterations = 60 ;
robot_posn = [5;5] ;
f1h=figure_named('costmap_dist_to_path_follower'); hold on; grid on; xlabel('x'); ylabel('y');
    daspect(  [  1 1 0.1  ]  );
    view(3) ;
f2h=figure_named('costmap_obs_dist '); hold on; grid on; xlabel('x'); ylabel('y');
    daspect(  [  1 1 0.1  ]  );
    view(3) ;
f3h=figure_named('costmap_obs_dist + costmap_dist_to_path_follower '); hold on; grid on; xlabel('x'); ylabel('y');
    daspect(  [  1 1 0.1  ]  );
    view(3) ;

main_task_start_point=[10;30]  ;
main_task_end_point=[60;30]  ;
path_follower_step=(main_task_end_point-main_task_start_point) / num_iterations  ;
path_follower_posn = main_task_start_point  ;
plot(main_task_end_point(1),main_task_end_point(2),'bo')

path_follower_posn_lagged = repmat(path_follower_posn,[1 5]);
robot_posn = path_follower_posn_lagged;

robot_start_posn = [ 60; 10 ]  ;
robot_end_posn = [ 10; 20 ]  ;
robot_step = ( robot_end_posn - robot_start_posn ) / num_iterations  ;
robot_target_posn = [100;90]  ;  %robot_start_posn  ;

max_dist = sqrt( ((1/floorplan_scale)*floorplan_y)^2 + ((1/floorplan_scale)*floorplan_x)^2 ) ;
max_cost = max_dist;

for t = 1:num_iterations
    path_follower_posn = path_follower_posn + path_follower_step  ;
    costmap_dist_to_path_follower = zeros((1/floorplan_scale)*floorplan_x,(1/floorplan_scale)*floorplan_y)  ;
    
    robot_target_posn = robot_target_posn + robot_step  ;
    
    for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
        for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
            cell_to_target =   - robot_target_posn  ;
            dist_cell_to_target = sqrt(   [ xx_- robot_target_posn(1) ]^2  +   [ yy_- robot_target_posn(2) ]^2   )  ;        
            costmap_obs_dist(xx_ , yy_) =  (0.00275*exp(dist_cell_to_target*0.1015*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
            if costmap_obs_dist(xx_ , yy_) > 1 
                costmap_obs_dist(xx_ , yy_) = 1  ;
            elseif costmap_obs_dist(xx_ , yy_) < 0  
                costmap_obs_dist(xx_ , yy_) = 0  ;
            end
        end
    end
    
    for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
        for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
            costmap_dist_to_path_follower(xx_,yy_) ...
                = sqrt(  (xx_ - path_follower_posn(1))^2  + (yy_ - path_follower_posn(2))^2)    /  max_dist;            
        end
    end
    
    figure(f1h)
    clf;
    grid on; hold on; 
    colormap hot  ;
    surf( costmap_dist_to_path_follower )  ;
    data_indicator_height = max(max(costmap_dist_to_path_follower)) ;
        plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
        plot3([path_follower_posn(2),path_follower_posn(2)],[path_follower_posn(1),path_follower_posn(1)],  [data_indicator_height , 0] , 'c', 'LineWidth',1) 
        text(  path_follower_posn(2) , path_follower_posn(1) , data_indicator_height   , 'aim', 'Color' , 'c')
            plot3_rows(  [   robot_posn(2) ; robot_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
            plot3([robot_posn(2),robot_posn(2)],[robot_posn(1),robot_posn(1)],  [data_indicator_height , 0] , 'g', 'LineWidth',1)   
            text(  robot_posn(2) , robot_posn(1) , data_indicator_height  , 'robot', 'Color'  , 'g')        
    plot3(robot_target_posn(2),robot_target_posn(1),  data_indicator_height , 'mx', 'LineWidth',5)   
    plot3([robot_target_posn(2),robot_target_posn(2)],[robot_target_posn(1),robot_target_posn(1)],  [ data_indicator_height ,0] , 'm', 'LineWidth',1) 
        view(3)
    
    figure(f2h)  ;
    clf;
    grid on; hold on; 
    colormap hot  ;
    surf( costmap_obs_dist )  ;  
    surf( costmap_dist_to_path_follower )  ;   
    data_indicator_height = max(max(costmap_obs_dist))  ;
        plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
        plot3([path_follower_posn(2),path_follower_posn(2)],[path_follower_posn(1),path_follower_posn(1)],  [data_indicator_height , 0] , 'c', 'LineWidth',1) 
        text(  path_follower_posn(2) , path_follower_posn(1) , data_indicator_height   , 'aim', 'Color' , 'c')
            plot3_rows(  [   robot_posn(2) ; robot_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
            plot3([robot_posn(2),robot_posn(2)],[robot_posn(1),robot_posn(1)],  [data_indicator_height , 0] , 'g', 'LineWidth',1)   
            text(  robot_posn(2) , robot_posn(1) , data_indicator_height  , 'robot', 'Color'  , 'g')  
    plot3(robot_target_posn(2),robot_target_posn(1),  data_indicator_height, 'mx', 'LineWidth',5)   
    plot3([robot_target_posn(2),robot_target_posn(2)],[robot_target_posn(1),robot_target_posn(1)],  [data_indicator_height,0] , 'm', 'LineWidth',1)   
        view(3)
    
    figure(f3h)  ;
    clf;
    grid on; hold on; 
    colormap hot  ;
    size(costmap_obs_dist)
    size(costmap_dist_to_path_follower)
    costmap_total = costmap_obs_dist + costmap_dist_to_path_follower ;
    surf( costmap_total )  ;
    data_indicator_height = max(max(costmap_total))  ;
    indicator_x = round(path_follower_posn(2));
    indicator_y = round(path_follower_posn(1));
    cost_at_indicator = costmap_total(  indicator_y , indicator_x )  ;
    robot_indicator_x = round(robot_posn(2));
    robot_indicator_y = round(robot_posn(1));
    robot_cost_at_indicator = costmap_total(  robot_indicator_y , robot_indicator_x )  ;
        plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
        plot3([path_follower_posn(2),path_follower_posn(2)],[path_follower_posn(1),path_follower_posn(1)],  [data_indicator_height , 0] , 'c', 'LineWidth',1) 
        text(  path_follower_posn(2) , path_follower_posn(1) , data_indicator_height   , 'aim', 'Color' , 'c')
            plot3_rows(  [   robot_posn(2) ; robot_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
            plot3([robot_posn(2),robot_posn(2)],[robot_posn(1),robot_posn(1)],  [data_indicator_height , 0] , 'g', 'LineWidth',1)   
            text(  robot_posn(2) , robot_posn(1) , data_indicator_height  , 'robot', 'Color'  , 'g') 
    plot3(robot_target_posn(2),robot_target_posn(1),  data_indicator_height , 'mx', 'LineWidth',5)   
    plot3([robot_target_posn(2),robot_target_posn(2)],[robot_target_posn(1),robot_target_posn(1)],  [data_indicator_height,0] , 'm', 'LineWidth',1)   
        obj_handle_ = patch([0 0 floorplan_extent_cells(2) floorplan_extent_cells(2) ],[floorplan_extent_cells(1)  0 0  floorplan_extent_cells(1)], [ cost_at_indicator cost_at_indicator cost_at_indicator cost_at_indicator ] , 'c');
        obj_handle_.FaceAlpha=0.1;
        obj_handle_ = patch([0 0 floorplan_extent_cells(2) floorplan_extent_cells(2) ],[floorplan_extent_cells(1)  0 0  floorplan_extent_cells(1)], [ robot_cost_at_indicator robot_cost_at_indicator robot_cost_at_indicator robot_cost_at_indicator ] , 'g');
        obj_handle_.FaceAlpha=0.1;
    view(3)
    
    pause  ;
    
    
    path_follower_posn_lagged(:,1) = path_follower_posn;
    path_follower_posn_lagged(:,2:5) = path_follower_posn_lagged(:,1:4);
    robot_posn = path_follower_posn_lagged(:,5);  % 2 step behind 
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

