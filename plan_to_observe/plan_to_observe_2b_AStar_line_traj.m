addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'
addpath(  '/mnt/nixbig/downloads/matlab_DistBetween2Segment/DistBetween2Segment/'  )

%{
Same as plan_to_observe_2b_AStar_cone.m,  but evaluate cost function just from the trajectory without the uncertainty 'cone'.
    AStar  -  
        path_planning__astar
        path_planning__reconstruct_path.m
        see /mnt/nixbig/ownCloud/project_code/plan_to_observe/path_planning__demo_astar.m
            [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
            [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
            path_planning__draw_came_from(came_from, strcat(test_name,': ',sprintf(' factor %7.f',factor_)))    ;    
            path_planning__draw_path( total_path_goal_to_start__ )    ;
        

%}

%% 
% Set up variables

floorplan_x = 11 % 7m forward
floorplan_y = 9 % 5m left
floorplan_extent_m = [floorplan_x;floorplan_y];
floorplan_scale = 0.1  % 10 cells/metre
floorplan_extent_cells = floorplan_extent_m.*(1/floorplan_scale)
floorplan_space = zeros(  ceil(floorplan_x*floorplan_scale) , ceil(floorplan_y*floorplan_scale)  ) ;
cost_to_divert = floorplan_space ;
cost_to_observe = floorplan_space ;

%%
% Ellipse: Mine:
%  x^2/a^2 + y^2/b^2 == 1  --> x=sqrt(a^2)*(1 - (y^2)/(b^2))  -->  y =   sqrt(( 1  -  x^2/a^2 ) * b^2  )

x=[-1:0.01:1]  ;
a=1;
b=0.5;
y = sqrt(( 1  -  (x.^2)/(a^2) ) * b^2  )

figure; hold on; plot(x,y); plot(x,-y); axis equal

%%
% Ellipse: https://au.mathworks.com/matlabcentral/answers/86615-how-to-plot-an-ellipse
a=5; % horizontal radius
b=10; % vertical radius
x0=0; % x0,y0 ellipse centre coordinates
y0=0;
t=-pi:0.01:pi;
x=x0+a*cos(t);
y=y0+b*sin(t);
plot(x,y)

%%
% Ellipse: https://au.mathworks.com/matlabcentral/answers/86615-how-to-plot-an-ellipse
x1 =10 ; y1 = 1 ;
x2 =  11 ; y2  = 3 ;
e = 0.99 ;

a = 1/2*sqrt((x2-x1)^2+(y2-y1)^2);
 b = a*sqrt(1-e^2);
 t = linspace(0,2*pi);
 X = a*cos(t);
 Y = b*sin(t);
 w = atan2(y2-y1,x2-x1);
 x = (x1+x2)/2 + X*cos(w) - Y*sin(w);
 y = (y1+y2)/2 + X*sin(w) + Y*cos(w);
 plot(x,y,'r-'); hold on
 axis equal
 min_idx = find(y==min(y))
 max_idx = find(y==max(y))
 min_x = x(min_idx)
 max_x = x(max_idx)
 
         rot=rotz(deg2rad(45));
         rot=rot(1:2,1:2)
        x_y = rot* ([x;y] - repmat(   [ x1+(x2-x1)/2 ;  y1+(y2-y1)/2 ] , 1 , size(x,2) ) )  + repmat(   [ x1+(x2-x1)/2 ;  y1+(y2-y1)/2 ] , 1 , size(x,2) );
     min_y = min(x_y(2,:))
     min_idx = find(x_y(2,:)==min_y)
     max_y = max(x_y(2,:))  
     max_idx = find(x_y(2,:)==max_y)
     min_x = x_y(1,min_idx)
     max_x = x_y(1,max_idx)

        plot(x_y(1,:),x_y(2,:),'b')
        plot(0 , 0 , 'bs')
        
 potential_zone = [ min_x  max_x  0;
   min_y  max_y  0] ;
potential_zone_closed = [ potential_zone potential_zone(:,1) ]
patch(potential_zone_closed(1,:),potential_zone_closed(2,:),'r')

%  NOTE:  gets me a cone/trapezoid and ellipses at each end:  would like a set of ellipses along a trajectory, and/or an envelope around a trajectory that's an
%  integration of the ellipses of probability --> 3D as is over time = configuration space 

%  How to check whether a point is inside an ellipse
%  How to check whether a line intersects an ellipse and where
 


%%
    % SAVE VARIABLES AND IMAGES - set up for this run
   
% temp for debug
debug_costmaps = false;
if debug_costmaps
    fig_costmap_obs_dist_orig = figure_named('fig_costmap_obs_dist_orig') ;
    fig_costmap_obs_dist_narrow= figure_named(' costmap_obs_dist_narrow ');
    fig_costmap_obs_dist= figure_named(' fig_costmap_obs_dist ');
end
%
    
    
    exp_name_script_name = 'plan_to_observe_2b_'  ;
    exp_run_start_time = datetime('now')  ;    
    exp_run_output_dir = strcat('/mnt/nixbig/ownCloud/project_code/plan_to_observe/',exp_name_script_name)  ;
    mkdir(exp_run_output_dir)  ;
    exp_run_output_dir = strcat('/mnt/nixbig/ownCloud/project_code/plan_to_observe/',exp_name_script_name,'/',datetostr(exp_run_start_time,'s'))  ;
    mkdir(exp_run_output_dir)  ;
    
    % RUN the thing
costmap_dist_to_path_follower = zeros(floorplan_extent_cells')  ;
size(costmap_dist_to_path_follower)  

num_iterations = 60 ;
loops = num_iterations*1;

draw_figs = true  ;    
% 
% if draw_figs
%     f1h=figure_named('costmap_dist_to_path_follower'); hold on; grid on; xlabel('x'); ylabel('y');
%         daspect(  [  1 1 0.1  ]  );
%         %  view(3) ;
%     f2h=figure_named('costmap_obs_dist '); hold on; grid on; xlabel('x'); ylabel('y');
%         daspect(  [  1 1 0.1  ]  );
%         %  view(3) ;
% end
f3h=figure_named('costmap_obs_dist + costmap_dist_to_path_follower '); 
sub_axis_h(1) = subplot(2,2,1);    
sub_axis_h(2) = subplot(2,2,2);   
sub_axis_h(3) = subplot(2,2,3);     
axes(sub_axis_h(1));   hold on; grid on; xlabel('x'); ylabel('y');
        daspect(  [  1 1 0.1  ]  );
%     pos = get( sub_axis_h(1), 'Position' )     ;   
        % the first two coordinates are the horizontal and vertical relative (in [0,1]) coordinates 
        %           of the origin of the axis 
        %           with respect to the lower left corner of the figure, 
        %  and the second two are the relative width and height.
                %     size=[pos(3) pos(4)];
                %     aspect=
                %     height=0.4; width=height
    sub_axis_h_Position{1} =  [  -0.05 0.05 0.5 0.9  ]  ;
    sub_axis_h(1).Position =  sub_axis_h_Position{1}  ;
    xlim([0,90]); ylim([0,110]);
    drawnow
        %     sub1_axis_h.Position =  [  0 0.6  pos(3) pos(4)  ]  ;    %        [pos(1) pos(2) pos(3) pos(4)]


axes(sub_axis_h(2));  hold on; grid on; xlabel('x'); ylabel('y');
        daspect(  [  1 1 0.1  ]  );
        view(3) ;
        sub_axis_h_Position{2} =  [  0.55 0.5  0.5 0.55  ]  ;
        sub_axis_h(2).Position =  sub_axis_h_Position{2}  ;
    xlim([0,90]); ylim([0,110]);
    drawnow
    
axes(sub_axis_h(3));  
        hold on; grid on; xlabel('x'); ylabel('y');
        daspect(  [  1 1 0.1  ]  );
        view(3) ;
        sub_axis_h_Position{3} =  [  0.35 0.05  0.55 0.55  ]  ;
        sub_axis_h(3).Position =  sub_axis_h_Position{3}  ;
    xlim([0,90]); ylim([0,110]);
    drawnow
    

costmap_obs_dist = ones(floorplan_extent_cells(1),floorplan_extent_cells(2))  ;
using_narrow_costmap = false; 
costmap_obs_dist_narrow = ones(floorplan_extent_cells(1),floorplan_extent_cells(2))  ; ;

    costmaps{1} = costmap_obs_dist;
    costmaps{2} = costmap_dist_to_path_follower;
    num_costmaps = size(costmaps,2)   ;

main_task_start_point=[10;30]  ;
main_task_end_point=[90;30]  ;
main_task_waypoints = [ main_task_start_point main_task_end_point ]  ;
main_task_current_waypoint_num = 1 + 1;
path_follower_step=(main_task_end_point-main_task_start_point) / num_iterations  ;
path_follower_posn = main_task_start_point  ;
plot(main_task_end_point(1),main_task_end_point(2),'bo')

path_follower_posn_lagged = repmat(path_follower_posn,[1 5]);

robot_posn = main_task_start_point ;

target_posn_start = [100;60]  ;  %robot_start_posn  ;
target_planned_path_start_posn = target_posn_start  ;
target_planned_path_end_posn = [ 10; 60 ]  ;
target_planned_path_step_base = ( target_planned_path_end_posn - target_planned_path_start_posn ) / num_iterations  ;
target_posn_prediction_vec = target_posn_start + target_planned_path_step_base  ;

max_dist = sqrt( ((1/floorplan_scale)*floorplan_y)^2 + ((1/floorplan_scale)*floorplan_x)^2 ) ;
max_cost = max_dist;
max_costmap_obs_dist=  0.5*(0.00275*exp(max_dist*0.07015*(3/4) ))   ; 


F(loops) = struct('cdata',[],'colormap',[]);

    costmap_obs_dist__hist = zeros([size(costmap_obs_dist) , loops])  ;
    costmap_dist_to_path_follower__hist = zeros([size(costmap_dist_to_path_follower) , loops])  ;
    costmap_obs_dist_narrow__hist = zeros([size(costmap_obs_dist_narrow) , loops])  ;
    robot_target_posn__hist = zeros(2,loops)  ;
    robot_posn__hist = zeros(2,loops)  ;
    path_follower_posn__hist = zeros(2,loops)  ;
    cost_at_robot_posn__hist = zeros(num_costmaps,loops);

    
% input('any key to start')    
pause(2)
    
t = 0 ;
while norm_2(main_task_end_point-robot_posn,1) > 0.01
    t = t + 1 ;
    path_follower_posn_old = path_follower_posn;
    path_follower_posn = path_follower_posn + path_follower_step  ;
    if sum(path_follower_posn> min(floorplan_extent_cells)) || sum(path_follower_posn<[1; 1])
        path_follower_posn = path_follower_posn_old  ;
    end
    path_follower_posn__hist(:,t) = path_follower_posn;
    costmap_dist_to_path_follower = zeros((1/floorplan_scale)*floorplan_x,(1/floorplan_scale)*floorplan_y)  ;
    costmap_total = zeros((1/floorplan_scale)*floorplan_x,(1/floorplan_scale)*floorplan_y)  ;
    
    robot_planned_path_step_noise_scale_factor = 0 ;
    target_posn_old = target_posn_start;        
    robot_planned_path_step_noise =  [randn(1) ; randn(1)].* (norm_2(target_planned_path_step_base,1)/10) ;
    robot_planned_path_step = target_planned_path_step_base + robot_planned_path_step_noise* robot_planned_path_step_noise_scale_factor;
    
    
    target_posn_start = target_posn_start + robot_planned_path_step  ;    
    target_posn_prediction_vec =  [ target_posn_start   target_posn_start+robot_planned_path_step*20]  ;
    %robot_posn_prediction_vec_1 = [ target_posn_start  ( target_posn_start+ (rot2(degtorad(+20)) * robot_planned_path_step*20) )  ];
    %robot_posn_prediction_vec_2 = [ target_posn_start  ( target_posn_start + (rot2(degtorad(-20)) * robot_planned_path_step*20) )  ];
    if sum(target_posn_start> floorplan_extent_cells) || sum(target_posn_start<[1; 1])
        target_posn_start = target_posn_old  ;
    end
    robot_target_posn__hist(:,t) =  target_posn_start  ;
    
    %   costmap_obs_dist
    
    for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
        for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
            cell_to_target =   - target_posn_start  ;
            %  dist_cell_to_target = sqrt(   [ xx_- robot_target_posn(1) ]^2  +   [ yy_- robot_target_posn(2) ]^2   )  ;        
             [line1_exit__, line2_exit__, line2_exit_to_line1_exit_vec__, dist__] = ...
                 shortestLineBetweenLineSegments(  ...
                 target_posn_prediction_vec(:,1) , target_posn_prediction_vec(:,2)  ,  ...
                 [xx_,yy_] , [xx_+0.01,yy_+0.01]  )  ;
             dist_cell_to_target = dist__ ;     
             
             dist_cell_to_target_radius_factor = 10 ;             
             dist_cell_to_target_radius_type = 'repel';
             
             if strcmpi('repel',dist_cell_to_target_radius_type )
                 if dist_cell_to_target < dist_cell_to_target_radius_factor
                     dist_cell_to_target = -dist_cell_to_target;
                 end
             end
            
            costmap_obs_dist(xx_ , yy_) =  0.75*(0.005275*exp(dist_cell_to_target*0.07015*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
            costmap_obs_dist(xx_ , yy_) =  costmap_obs_dist(xx_ , yy_) * 1.1  ;
            
             if strcmpi('flat',dist_cell_to_target_radius_type )
                 if dist_cell_to_target < dist_cell_to_target_radius_factor
                     costmap_obs_dist(xx_ , yy_) =  0 ;
                 end
             end
            
            if costmap_obs_dist(xx_ , yy_) > 1 
                costmap_obs_dist(xx_ , yy_) = 1  ;
            elseif costmap_obs_dist(xx_ , yy_) < 0  
                costmap_obs_dist(xx_ , yy_) = 0  ;
            end
        end
    end
    %     costmap_obs_dist = costmap_obs_dist .*0.5  ;
            if using_narrow_costmap; 
            for yy_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_y
                for xx_ = 1.0 : 1 : (1/floorplan_scale)*floorplan_x
                    cell_to_target =   - target_posn_start  ;
                    %  dist_cell_to_target = sqrt(   [ xx_- robot_target_posn(1) ]^2  +   [ yy_- robot_target_posn(2) ]^2   )  ;        
                    [line1_exit__, line2_exit__, line2_exit_to_line1_exit_vec__, dist__] = ...
                        shortestLineBetweenLineSegments(  ...
                        target_posn_prediction_vec(:,1) , target_posn_prediction_vec(:,2)  ,  ...
                        [xx_,yy_] , [xx_,yy_]  )  ;
                    dist_cell_to_target = dist__ ;
                     %  [line1_exit__, line2_exit__, line2_exit_to_line1_exit_vec__, dist__] = shortestLineBetweenLineSegments( line1_pt1_, line1_pt2_, line2_pt1_, line2_pt2_)           

                    costmap_obs_dist_narrow(xx_ , yy_) =  0.75*(0.00275*exp(dist_cell_to_target*0.17015*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
                    if costmap_obs_dist_narrow(xx_ , yy_) > 1 
                        costmap_obs_dist_narrow(xx_ , yy_) = 1  ;
                    elseif costmap_obs_dist_narrow(xx_ , yy_) < 0  
                        costmap_obs_dist_narrow(xx_ , yy_) = 0  ;
                    end
                end
            end
            end
    
%     costmap_obs_dist_narrow = costmap_obs_dist_narrow .* 0.5  ;
   % costmap_obs_dist = costmap_obs_dist_narrow.*costmap_obs_dist;
    

        if debug_costmaps ;  figure(fig_costmap_obs_dist_orig);  surf(costmap_obs_dist); end;
        if debug_costmaps;  figure(fig_costmap_obs_dist_narrow);  surf(costmap_obs_dist); if using_narrow_costmap;  surf(costmap_obs_dist_narrow);    end;    end;    
    
    costmap_obs_dist_scale_factor = 0.001;
    costmap_obs_dist = costmap_obs_dist .*  costmap_obs_dist_scale_factor;    
    
        if debug_costmaps;  if using_narrow_costmap;  figure(fig_costmap_obs_dist);  surf(costmap_obs_dist); surf(costmap_obs_dist_narrow);   end; end;
    
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
    subplot(sub_axis_h(2));    
    cla();
    grid on; hold on; 
    colormap hot  ;
        sub_axis_h(2).Position =  sub_axis_h_Position{2}  ;
    surf( costmap_dist_to_path_follower )  ;
    surf( costmap_total )  ;
    data_indicator_height = 1.1; %  max(max(costmap_dist_to_path_follower)) ;
        plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
        plot3([path_follower_posn(2),path_follower_posn(2)],[path_follower_posn(1),path_follower_posn(1)],  [data_indicator_height , 0] , 'c', 'LineWidth',1) 
        text(  path_follower_posn(2) , path_follower_posn(1) , data_indicator_height   , 'aim', 'Color' , 'c')
                plot3( main_task_waypoints(2,:)  , main_task_waypoints(1,:), [data_indicator_height data_indicator_height], 'c:', 'LineWidth',2)   ;
            plot3_rows(  [   robot_posn(2) ; robot_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
            plot3([robot_posn(2),robot_posn(2)],[robot_posn(1),robot_posn(1)],  [data_indicator_height , 0] , 'g', 'LineWidth',1)   
            text(  robot_posn(2) , robot_posn(1) , data_indicator_height  , 'robot', 'Color'  , 'g')        
    plot3(target_posn_start(2),target_posn_start(1),  data_indicator_height , 'mx', 'LineWidth',5)   
    plot3([target_posn_start(2),target_posn_start(2)],[target_posn_start(1),target_posn_start(1)],  [ data_indicator_height ,0] , 'm', 'LineWidth',1) 
    plot3([target_posn_prediction_vec(2,1),target_posn_prediction_vec(2,2)],[target_posn_prediction_vec(1,1),target_posn_prediction_vec(1,2)],  [data_indicator_height,data_indicator_height] , 'm', 'LineWidth',1)   
    %plot3([robot_posn_prediction_vec_1(2,1),robot_posn_prediction_vec_1(2,2)],[robot_posn_prediction_vec_1(1,1),robot_posn_prediction_vec_1(1,2)],  [data_indicator_height,data_indicator_height] , 'm:', 'LineWidth',1)   
    %plot3([robot_posn_prediction_vec_2(2,1),robot_posn_prediction_vec_2(2,2)],[robot_posn_prediction_vec_2(1,1),robot_posn_prediction_vec_2(1,2)],  [data_indicator_height,data_indicator_height] , 'm:', 'LineWidth',1)   
    daspect(  [  1 1 0.025   ]  );
    zlim([0 1.1])  ;
    view(3)
    xlim([0,90]); ylim([0,110]);
    
    
    subplot(sub_axis_h(3));    
    cla();
    grid on; hold on; 
    colormap hot  ;
        sub_axis_h(3).Position =  sub_axis_h_Position{3}  ;
    surf( costmap_obs_dist )  ;  
    surf( costmap_dist_to_path_follower )  ;   
    data_indicator_height = 1.1; %    max(max(costmap_obs_dist))  ;
        plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
        plot3([path_follower_posn(2),path_follower_posn(2)],[path_follower_posn(1),path_follower_posn(1)],  [data_indicator_height , 0] , 'c', 'LineWidth',1) 
        text(  path_follower_posn(2) , path_follower_posn(1) , data_indicator_height   , 'aim', 'Color' , 'c')
                plot3( main_task_waypoints(2,:)  , main_task_waypoints(1,:), [data_indicator_height data_indicator_height], 'c:', 'LineWidth',2)   ;
            plot3_rows(  [   robot_posn(2) ; robot_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
            plot3([robot_posn(2),robot_posn(2)],[robot_posn(1),robot_posn(1)],  [data_indicator_height , 0] , 'g', 'LineWidth',1)   
            text(  robot_posn(2) , robot_posn(1) , data_indicator_height  , 'robot', 'Color'  , 'g')  
    plot3(target_posn_start(2),target_posn_start(1),  data_indicator_height, 'mx', 'LineWidth',5)   
    plot3([target_posn_start(2),target_posn_start(2)],[target_posn_start(1),target_posn_start(1)],  [data_indicator_height,0] , 'm', 'LineWidth',1)   
    plot3([target_posn_prediction_vec(2,1),target_posn_prediction_vec(2,2)],[target_posn_prediction_vec(1,1),target_posn_prediction_vec(1,2)],  [data_indicator_height,data_indicator_height] , 'm', 'LineWidth',1)   
    %plot3([robot_posn_prediction_vec_1(2,1),robot_posn_prediction_vec_1(2,2)],[robot_posn_prediction_vec_1(1,1),robot_posn_prediction_vec_1(1,2)],  [data_indicator_height,data_indicator_height] , 'm:', 'LineWidth',1)   
    %plot3([robot_posn_prediction_vec_2(2,1),robot_posn_prediction_vec_2(2,2)],[robot_posn_prediction_vec_2(1,1),robot_posn_prediction_vec_2(1,2)],  [data_indicator_height,data_indicator_height] , 'm:', 'LineWidth',1)   
    
    daspect(  [  1 1 0.025   ]  )  ;
    zlim([0 1.1])  ;
    view(3)
    xlim([0,90]); ylim([0,110]);
    end
    
    costmaps{1} = costmap_obs_dist;
    costmaps{2} = costmap_dist_to_path_follower;
    SAMPLE_VALUE_per_costmap = zeros(num_costmaps,1,'double');
    min_SAMPLE_VALUE_per_costmap = ones(1,num_costmaps,'double').*10000;
    min_SAMPLE_dir_yx_per_costmap = zeros(2,num_costmaps,'double');
    
    %  figure(f3h)  ;
    subplot(sub_axis_h(1));    
    cla();
    grid on; hold on; 
    %colormap hot  ;
        sub_axis_h(1).Position =  sub_axis_h_Position{1}  ;
    size(costmap_obs_dist)
    size(costmap_dist_to_path_follower)
    
    % for local-sampling-based
    costmap_obs_dist_max = max(max(costmap_obs_dist));
    costmap_total =  costmap_dist_to_path_follower +  ( costmap_obs_dist - costmap_obs_dist_max )  ;
    costmap_total_min = min(min(costmap_total))  ;
    costmap_total = costmap_total + costmap_total_min  ;
    
    % for straight A*
    costmap_total = costmap_obs_dist;
    costmap_total =  costmap_total + 1.0;  % do NOT know why I need this : need to add at least 1.0 to get A* to plan 
    
    
    surf( costmap_total )  ;
    data_indicator_height = 1.45; %1.5;  %max(max(costmap_total))  ;
    indicator_x = round(path_follower_posn(2));
    indicator_y = round(path_follower_posn(1));
    cost_at_indicator = costmap_total(  indicator_y , indicator_x )  ;
    robot_indicator_x = round(robot_posn(2));
    robot_indicator_y = round(robot_posn(1));
    robot_cost_at_indicator = costmap_total(  robot_indicator_y , robot_indicator_x )  ;
    display('362')
        plot3_rows(  [   path_follower_posn(2) ; path_follower_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
        plot3([path_follower_posn(2),path_follower_posn(2)],[path_follower_posn(1),path_follower_posn(1)],  [data_indicator_height , 0] , 'c', 'LineWidth',1) 
        text(  path_follower_posn(2) , path_follower_posn(1) , data_indicator_height   , 'aim', 'Color' , 'c')
                plot3( main_task_waypoints(2,:)  , main_task_waypoints(1,:), [data_indicator_height data_indicator_height], 'c:', 'LineWidth',2)   ;
            plot3_rows(  [   robot_posn(2) ; robot_posn(1)  ;  data_indicator_height ] , 'cx'  , 'LineWidth',5)
            plot3([robot_posn(2),robot_posn(2)],[robot_posn(1),robot_posn(1)],  [data_indicator_height , 0] , 'g', 'LineWidth',1)   
            
              
                

    %  start next step calculation                
            display('start next step calculation');
            map_1 = costmap_total;
            start_1 = robot_posn  ;
            goal_1 = [  main_task_waypoints(1,main_task_current_waypoint_num) ;  main_task_waypoints(2,main_task_current_waypoint_num) ] ;
            %             goal_1 = [ 90 ; 30 ]  ;
            % A*
                % note:  DStarMoo :  incorporate the orientation of target and orientation of uncertainty 
            [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
            [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
            dir_vec_end_yx = total_path_start_to_goal__(:,2)   ;
            
            %             hold on; plot3(start_1(1),goal_1(2), 3,'rs','LineWidth',5)
            %             hold on; plot3(start_1(1),start_1(2), 3,'md','LineWidth',5)
            plot3(total_path_start_to_goal__(2,:),total_path_start_to_goal__(1,:),map_1(total_path_start_to_goal__(2,:),total_path_start_to_goal__(1,:)),'ro')
            
            display('end next step calculation');
    %  end next step calculation
            
    plot3(target_posn_start(2),target_posn_start(1),  data_indicator_height , 'mx', 'LineWidth',5)   
    plot3([target_posn_start(2),target_posn_start(2)],[target_posn_start(1),target_posn_start(1)],  [data_indicator_height,0] , 'm', 'LineWidth',1)   
    plot3([target_posn_prediction_vec(2,1),target_posn_prediction_vec(2,2)],[target_posn_prediction_vec(1,1),target_posn_prediction_vec(1,2)],  [data_indicator_height,data_indicator_height] , 'm', 'LineWidth',1)   
    %plot3([robot_posn_prediction_vec_1(2,1),robot_posn_prediction_vec_1(2,2)],[robot_posn_prediction_vec_1(1,1),robot_posn_prediction_vec_1(1,2)],  [data_indicator_height,data_indicator_height] , 'm', 'LineWidth',1)   
    %plot3([robot_posn_prediction_vec_2(2,1),robot_posn_prediction_vec_2(2,2)],[robot_posn_prediction_vec_2(1,1),robot_posn_prediction_vec_2(1,2)],  [data_indicator_height,data_indicator_height] , 'm', 'LineWidth',1)   
        

    
    
    path_follower_posn_lagged(:,1) = path_follower_posn;
    path_follower_posn_lagged(:,2:5) = path_follower_posn_lagged(:,1:4);
%     robot_posn = path_follower_posn_lagged(:,5);  % 2 step behind 
    
%     robot_posn 
    robot_posn = dir_vec_end_yx;
    robot_posn__hist(:,t) = robot_posn  ;
	
    robot_indicator_x = round(robot_posn(2));
    robot_indicator_y = round(robot_posn(1));
    
    for ii_ = 1:size(costmaps,2)
        cost_at_robot_posn = costmaps{ii_}(robot_indicator_y,robot_indicator_x)  ;
        cost_at_robot_posn__hist(ii_,t) = cost_at_robot_posn  ;
    end
	
    path_altitude = repmat(data_indicator_height, 1, size(robot_posn__hist,2))  ;
    plot3(robot_target_posn__hist(2,:) , robot_target_posn__hist(1,:),  path_altitude, 'mx' ) ;
    plot3(path_follower_posn__hist(2,:) , path_follower_posn__hist(1,:), path_altitude  , 'co') ;
    plot3(path_follower_posn__hist(2,:) , path_follower_posn__hist(1,:), path_altitude  , 'co') ;
    plot3(robot_posn__hist(2,:) , robot_posn__hist(1,:),  path_altitude, 'bs' ) ;
    %         obj_handle_ = patch([0 0 floorplan_extent_cells(2) floorplan_extent_cells(2) ],[floorplan_extent_cells(1)  0 0  floorplan_extent_cells(1)], [ cost_at_indicator cost_at_indicator cost_at_indicator cost_at_indicator ] , 'c');
    %         obj_handle_.FaceAlpha=0.1;
    %         obj_handle_ = patch([0 0 floorplan_extent_cells(2) floorplan_extent_cells(2) ],[floorplan_extent_cells(1)  0 0  floorplan_extent_cells(1)], [ robot_cost_at_indicator robot_cost_at_indicator robot_cost_at_indicator robot_cost_at_indicator ] , 'g');
    %         obj_handle_.FaceAlpha=0.1;
        % daspect(  [  1 1 0.05  ]  );
        daspect(  [  1 1 0.025  ]  );
    %  view(3) ;
    xlim([0,90]); ylim([0,110]);
    
    % SAVE VARIABLES AND IMAGES   

    
    drawnow
    
    costmap_obs_dist__hist(:,:,t) = costmap_obs_dist  ;
    costmap_dist_to_path_follower__hist(:,:,t) = costmap_dist_to_path_follower  ;
    if using_narrow_costmap; costmap_obs_dist_narrow__hist(:,:,t) = costmap_obs_dist_narrow  ; end;
    
    pause(0.1)
end
display('At end of simulation!')
% v = VideoWriter(strcat(exp_run_output_dir,'/','f3h.mp4'))  ;
% open(v)  ;
% writeVideo(v,A)  ;
% close(v)  ;
exp_run_start_time.Format
save(  strcat(exp_run_output_dir,'/',exp_name_script_name,datetostr(exp_run_start_time),'.m')  )

display('Displaying summaries')

figure_named('Timestep'); grid on; xlabel('time'); hold on; 
benefit_ = 1-cost_at_robot_posn__hist(1,:)  ;
plot(benefit_)  ;
plot(cost_at_robot_posn__hist(2,:))  ;
net_gain = (1-cost_at_robot_posn__hist(2,:))-cost_at_robot_posn__hist(1,:)  ;
plot( net_gain )  ;
legend('benefit of observation','main mission cost = cost of distance from path','net gain (unscaled)')  ;

figure_named('Cummulative'); grid on; xlabel('time'); hold on; 
plot(cumsum(benefit_))  ;
plot(cumsum(cost_at_robot_posn__hist(2,:)))  ;
plot( cumsum( net_gain ))  ;
legend('benefit of observation','main mission cost = cost of distance from path','net gain (unscaled)')  ;

display('Ended!')

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

