addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'

%{
Same as plan_to_observe_2.m, but 
%}

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
surf( costmap_obs_dist )  ;  %  view(3)  ;  
draw_axes_direct(eye(3),[floorplan_extent_cells(1)/2;floorplan_extent_cells(2)/2;0],'',1)
axes.DataAspectRatio=[1 1 0.02]


axes=subplot(1,2 , 2);  colormap hot; title('Observation benefit/gain/measurement model falls with distance') ;  hold on; grid on; xlabel('x (0.1m)'); ylabel('y (0.1m)'); zlabel('cost')
surf( 1-costmap_obs_dist )  ;  %  view(3)  ;
draw_axes_direct(eye(3),[floorplan_extent_cells(1)/2;floorplan_extent_cells(2)/2;0],'',1)
axes.DataAspectRatio=[1 1 0.02]


%%
costmap_dist_to_path_follower = zeros(floorplan_extent_cells')  ;
size(costmap_dist_to_path_follower)  

num_iterations = 60 ;

draw_figs = false  ;    

if draw_figs
    f1h=figure_named('costmap_dist_to_path_follower'); hold on; grid on; xlabel('x'); ylabel('y');
        daspect(  [  1 1 0.1  ]  );
        %  view(3) ;
    f2h=figure_named('costmap_obs_dist '); hold on; grid on; xlabel('x'); ylabel('y');
        daspect(  [  1 1 0.1  ]  );
        %  view(3) ;
end
f3h=figure_named('costmap_obs_dist + costmap_dist_to_path_follower '); hold on; grid on; xlabel('x'); ylabel('y');
    daspect(  [  1 1 0.1  ]  );
    %  view(3) ;
    

costmap_obs_dist = ones(floorplan_extent_cells(1),floorplan_extent_cells(2))  ;
costmap_obs_dist_narrow = ones(floorplan_extent_cells(1),floorplan_extent_cells(2))  ;

main_task_start_point=[10;30]  ;
main_task_end_point=[90;30]  ;
path_follower_step=(main_task_end_point-main_task_start_point) / num_iterations  ;
path_follower_posn = main_task_start_point  ;
plot(main_task_end_point(1),main_task_end_point(2),'bo')

path_follower_posn_lagged = repmat(path_follower_posn,[1 5]);

robot_posn = main_task_start_point ;

robot_target_posn = [100;90]  ;  %robot_start_posn  ;
robot_planned_path_start_posn = robot_target_posn  ;
robot_planned_path_end_posn = [ 10; 90 ]  ;
robot_planned_path_step = ( robot_planned_path_end_posn - robot_planned_path_start_posn ) / num_iterations  ;

max_dist = sqrt( ((1/floorplan_scale)*floorplan_y)^2 + ((1/floorplan_scale)*floorplan_x)^2 ) ;
max_cost = max_dist;
max_costmap_obs_dist=  0.5*(0.00275*exp(max_dist*0.07015*(3/4) ))   ; 

for t = 1:num_iterations*5
    path_follower_posn_old = path_follower_posn;
    path_follower_posn = path_follower_posn + path_follower_step  ;
    if sum(path_follower_posn> min(floorplan_extent_cells)) || sum(path_follower_posn<[1; 1])
        path_follower_posn = path_follower_posn_old  ;
    end
    costmap_dist_to_path_follower = zeros((1/floorplan_scale)*floorplan_x,(1/floorplan_scale)*floorplan_y)  ;
    
    robot_target_posn_old = robot_target_posn;
    robot_target_posn = robot_target_posn + robot_planned_path_step  ;
    if sum(robot_target_posn> floorplan_extent_cells) || sum(robot_target_posn<[1; 1])
        robot_target_posn = robot_target_posn_old  ;
    end
    
    %   costmap_obs_dist
    for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
        for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
            cell_to_target =   - robot_target_posn  ;
            dist_cell_to_target = sqrt(   [ xx_- robot_target_posn(1) ]^2  +   [ yy_- robot_target_posn(2) ]^2   )  ;        
            costmap_obs_dist(xx_ , yy_) =  0.75*(0.00275*exp(dist_cell_to_target*0.07015*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
            costmap_obs_dist(xx_ , yy_) =  costmap_obs_dist(xx_ , yy_) * 1.1  ;
            if costmap_obs_dist(xx_ , yy_) > 1 
                costmap_obs_dist(xx_ , yy_) = 1  ;
            elseif costmap_obs_dist(xx_ , yy_) < 0  
                costmap_obs_dist(xx_ , yy_) = 0  ;
            end
        end
    end
    %     costmap_obs_dist = costmap_obs_dist .*0.5  ;
    for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
        for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
            cell_to_target =   - robot_target_posn  ;
            dist_cell_to_target = sqrt(   [ xx_- robot_target_posn(1) ]^2  +   [ yy_- robot_target_posn(2) ]^2   )  ;        
            costmap_obs_dist_narrow(xx_ , yy_) =  0.75*(0.00275*exp(dist_cell_to_target*0.17015*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
            if costmap_obs_dist_narrow(xx_ , yy_) > 1 
                costmap_obs_dist_narrow(xx_ , yy_) = 1  ;
            elseif costmap_obs_dist_narrow(xx_ , yy_) < 0  
                costmap_obs_dist_narrow(xx_ , yy_) = 0  ;
            end
        end
    end
%     costmap_obs_dist_narrow = costmap_obs_dist_narrow .* 0.5  ;
    costmap_obs_dist = costmap_obs_dist_narrow.*costmap_obs_dist;
    
    %   costmap_dist_to_path_follower
    for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
        for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
            dist_ = sqrt(  (xx_ - path_follower_posn(1))^2  + (yy_ - path_follower_posn(2))^2);
            if dist_ <= 75
                costmap_dist_to_path_follower(xx_,yy_) = (dist_    /  max_dist )^2  +  (75/max_dist)-(75/max_dist)^2;                
            else                       
                costmap_dist_to_path_follower(xx_,yy_)  = dist_    /  max_dist   ;            
            end
        end
    end
    costmap_dist_to_path_follower = costmap_dist_to_path_follower .* 0.6 ;
    
%     costmap_dist_to_path_follower(costmap_dist_to_path_follower>0.2)  ;
    
    tolerance = 2 ; %2m
    tolerance_cells = round(tolerance* (1/floorplan_scale))  ;
    tolerance_cost_level = costmap_dist_to_path_follower(1,tolerance_cells)  ;    
    costmap_dist_to_path_follower_2 = costmap_dist_to_path_follower  ;
        exponent_here=1.7;
    costmap_dist_to_path_follower(costmap_dist_to_path_follower<tolerance_cost_level) ...
        = costmap_dist_to_path_follower(costmap_dist_to_path_follower<tolerance_cost_level).^exponent_here  ...
            +   max(max(costmap_dist_to_path_follower(costmap_dist_to_path_follower==tolerance_cost_level))) -   max(max(costmap_dist_to_path_follower(costmap_dist_to_path_follower==tolerance_cost_level)))^exponent_here     ; 

    if draw_figs
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
        %  view(3)
    
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
        %  view(3)
    end
    
    costmaps{1} = costmap_obs_dist;
    costmaps{2} = costmap_dist_to_path_follower;
    num_costmaps = size(costmaps,2)   ;
    SAMPLE_VALUE_per_costmap = zeros(num_costmaps,1,'double');
    min_SAMPLE_VALUE_per_costmap = ones(1,num_costmaps,'double').*10000;
    min_SAMPLE_dir_yx_per_costmap = zeros(2,num_costmaps,'double');
    
    figure(f3h)  ;
    clf;
    grid on; hold on; 
    colormap hot  ;
    size(costmap_obs_dist)
    size(costmap_dist_to_path_follower)
    costmap_total = costmap_obs_dist + costmap_dist_to_path_follower ;
    surf( costmap_total )  ;
    data_indicator_height = 1.5;  %max(max(costmap_total))  ;
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
                min_sample = robot_cost_at_indicator*100;     
                min_sample_SAMPLE_Y=0;
                min_sample_SAMPLE_X=0;
                for xii_ = [ -1 -0.5 0 0.5 1 ].*5
                    for yii_ = [ 1 0.5 0  -0.5 -1].*5
                        if xii_ == yii_ ;   continue;     end;                            
                        SAMPLE_Y = robot_indicator_x+round(yii_)  ;
                        SAMPLE_X = robot_indicator_y+round(xii_)  ;
                        SAMPLE_VALUE = costmap_total(SAMPLE_X,SAMPLE_Y)  ;
                        if SAMPLE_VALUE< min_sample
                            min_sample = SAMPLE_VALUE  ;
                            min_sample_SAMPLE_Y = SAMPLE_Y ;
                            min_sample_SAMPLE_X = SAMPLE_X ;
                        end
                        for cc_ = 1:size(costmaps,2)                            
                            SAMPLE_VALUE_per_costmap(cc_) = costmaps{cc_}(SAMPLE_X,SAMPLE_Y)  ;
                            if SAMPLE_VALUE_per_costmap(cc_) < min_SAMPLE_VALUE_per_costmap(cc_)
                                min_SAMPLE_VALUE_per_costmap(cc_) = SAMPLE_VALUE_per_costmap(cc_)  ;
                                min_SAMPLE_dir_yx_per_costmap(:,cc_) = [SAMPLE_Y;SAMPLE_X]  ;
                            end
                        end
                        plot3( [SAMPLE_Y , SAMPLE_Y]  ,  [SAMPLE_X, SAMPLE_X],  [data_indicator_height , 0] , 'y', 'LineWidth',1)   
                        colour='r';    
                        diff_ = SAMPLE_VALUE - robot_cost_at_indicator  ;
                        if  diff_ <= 0.025
                            colour=[0 .4 1] ;    
                        elseif  diff_ <= 0
                            colour=[0 .7 0] ;    
                        elseif   diff_ <= 0.025
                            colour=[0 .7 .2] ;        
                        elseif   diff_ <=  0.05
                            colour=[0 .7 .4] ;        
                        end  ;
                         p1_pt_from =  [robot_posn(2);robot_posn(1);data_indicator_height]  ;
                         p2_pt_to = [SAMPLE_Y ; SAMPLE_X ; data_indicator_height]  ;
%                          if sum(abs(p1_pt_from - p2_pt_to)) >= 0.01 
%                             %arrow3(  p1_pt_from , p2_pt_to  , colour  ) % takes points as row data  :  arrow3(p1,p2,s,w,h,ip,alpha,beta)  :  arrow3(pts_from_nx3,pts_to_nx3, colour+style+width,head_widths,head_heights,head_lengths?,  alpha transparency ,beta colour shade range)
                         plot3_rows([p1_pt_from p2_pt_to], 'Color',colour)
%                          end
                    end
                end
                
                % kind of vote:  sum across the x and y 
                dir_vec_end_yx = [0;0];
                for cc_ = 1:size(costmaps,2)     
                    dir_vec_end_yx = dir_vec_end_yx + flip(min_SAMPLE_dir_yx_per_costmap(:,cc_)) -  robot_posn ;        
                    min_SAMPLE_dir_yx_per_costmap(:,cc_)  
                     p1_pt_from =  [robot_posn(2);robot_posn(1);data_indicator_height+0.0001]  ;
                     p2_pt_to = [min_SAMPLE_dir_yx_per_costmap(:,cc_)   ; data_indicator_height+0.0001]  ;
                     plot3_rows([p1_pt_from p2_pt_to], 'm', 'LineWidth',2)  ;
                end
                priority_costmap = 2;
                norm_2(dir_vec_end_yx,1)
                if norm_2(dir_vec_end_yx,1) <= 3
                    dir_vec_end_yx
                    dir_vec_end_yx = dir_vec_end_yx + flip(min_SAMPLE_dir_yx_per_costmap(:,priority_costmap)) -  robot_posn ;  
                    dir_vec_end_yx
                end
                dir_vec_end_yx = dir_vec_end_yx./norm_2(dir_vec_end_yx,1)   ;                
                
                if min_sample < robot_cost_at_indicator 
                    plot3_rows(  [   min_sample_SAMPLE_Y ; min_sample_SAMPLE_X  ;  data_indicator_height ] , 'bx'  , 'LineWidth',5)                    
                end
            
    plot3(robot_target_posn(2),robot_target_posn(1),  data_indicator_height , 'mx', 'LineWidth',5)   
    plot3([robot_target_posn(2),robot_target_posn(2)],[robot_target_posn(1),robot_target_posn(1)],  [data_indicator_height,0] , 'm', 'LineWidth',1)   
    %         obj_handle_ = patch([0 0 floorplan_extent_cells(2) floorplan_extent_cells(2) ],[floorplan_extent_cells(1)  0 0  floorplan_extent_cells(1)], [ cost_at_indicator cost_at_indicator cost_at_indicator cost_at_indicator ] , 'c');
    %         obj_handle_.FaceAlpha=0.1;
    %         obj_handle_ = patch([0 0 floorplan_extent_cells(2) floorplan_extent_cells(2) ],[floorplan_extent_cells(1)  0 0  floorplan_extent_cells(1)], [ robot_cost_at_indicator robot_cost_at_indicator robot_cost_at_indicator robot_cost_at_indicator ] , 'g');
    %         obj_handle_.FaceAlpha=0.1;
        % daspect(  [  1 1 0.05  ]  );
        daspect(  [  1 1 0.025  ]  );
    %  view(3) ;
    
    pause  ;    
    
    path_follower_posn_lagged(:,1) = path_follower_posn;
    path_follower_posn_lagged(:,2:5) = path_follower_posn_lagged(:,1:4);
%     robot_posn = path_follower_posn_lagged(:,5);  % 2 step behind 
    
%     robot_posn 
    if true
             robot_next_step = dir_vec_end_yx   ; 
             robot_next_step = robot_next_step .* norm_2(path_follower_step,1)/norm_2(robot_next_step,1)   * 1.0 ; 
             robot_posn = robot_posn + robot_next_step; 
    else
         if min_sample < robot_cost_at_indicator                      
             robot_next_step = ([ min_sample_SAMPLE_X;min_sample_SAMPLE_Y ] - robot_posn)  ;
             robot_next_step = robot_next_step .* norm_2(path_follower_step,1)/norm_2(robot_next_step,1)   * 1.2 ; 
             robot_posn = robot_posn + robot_next_step;         
         end
    end
    
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

