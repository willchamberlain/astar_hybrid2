%%  Sensor model : 
%       uncertainty vs. distance of feature to camera optical centre
%       uncertainty vs. angle of robot feature to camera optical axis
%%
addpath('/mnt/nixbig/ownCloud/project_code/')
addpath('/mnt/nixbig/ownCloud/project_code/plan_to_observe/')
addpath('/mnt/nixbig/ownCloud/project_code/3rd_party/robotics-toolbox-matlab')
%%

floorplan_image_file_data = load_map_from_ROS_gmapping( '/mnt/nixbig/data/project_AA1_backup_pioneer3c/lvl11_map2/map_small.pgm' , 0.050000  ) ;
floorplan = flip(floorplan_image_file_data,1) ;
size(floorplan)
f_floorplan = figure; idisp(floorplan, 'ynormal')

floorplan = iopen(floorplan,ones(4),5) ;        %  image-specific 
size(floorplan)
f_floorplan_opened = figure; idisp(floorplan,'ynormal')





%%  (1) Camera position in the world

%-- coordinate and measurement conventions
scale_up = 10  ;  scale_down = 1/scale_up  ;
scale_up_ = scale_up  ;
scale_down_from_ROS_map = 0.1  ;
scale_up_from_ROS_map = 1/scale_down_from_ROS_map  ;

pixels_per_metre = 100;
cells_per_metre = 10 ;
cells_per_pixel = cells_per_metre / pixels_per_metre  ;
pixels_per_cell = 1/cells_per_pixel  ;

ROS_to_image_ = [ 0 1 ; 1 0 ]  ;

image_to_world__cob = ...  %  cob = change of basis 
    [   1/pixels_per_metre   0
        0                                   1/pixels_per_metre  ] ;

%-- world 
world_x_extent_cells = [ 1  size(floorplan,2)/pixels_per_cell  ]  ;  % size(floorplan,2) because looking at it as an image:  x --> right
world_y_extent_cells = [ 1  size(floorplan,1)/pixels_per_cell ]   ;  % size(floorplan,2) because looking at it as an image:  y --> down or up, depending on e.g. 'ynormal' 

offset_pixels = [ 710 ; 350 ]  ;


%% (1) Main section - add two more cameras, calculate information gain/uncertainty reduction 

%  uncertainty_vs_distance = (0.714285714285714*0.0005*exp(dist))                                                   


%  SET UP THE CAMERAS

%-----  First camera 
x_unit_generic = [1;0]  ;
screen_to_ros = [ ...
    0 1
    -1 0 ]  ;
ros_to_screen = inv(screen_to_ros)  ;
x_unit_ros__screen = ros_to_screen*x_unit_generic  ;

x_unit_screen__screen = [ ...
    1
    0 ]  ;

camera_1_angle_yaw__screen = 0  ;
camera_position_1_image     = [ 730 ; 325 ]  ;  % ;degtorad(camera_1_angle_yaw)]  ;
camera_position_1_world     =  image_to_world__cob * camera_position_1_image  ;
camera_position_1_cells     = camera_position_1_world.*cells_per_metre  ;
camera_angle_1_rot2         = rotz(degtorad(camera_1_angle_yaw__screen))  ;     camera_angle_1_rot2=camera_angle_1_rot2(1:2,1:2)  ;
camera_optical_axis_direction_unit_1 = camera_angle_1_rot2*x_unit_screen__screen  ;
FoV_angle_1                         = 62 ;
[ distance_uncertainty_costs_1 , check_dist_map , check_angle_map ] ...
    = sensor_model_func(scale_up_, world_x_extent_cells, world_y_extent_cells, camera_optical_axis_direction_unit_1 , camera_position_1_cells , FoV_angle_1)  ;
distance_uncertainty_costs_1 = distance_uncertainty_costs_1'  ;
%-----  Second camera 
camera_2_angle_yaw__screen = 45  ;
camera_position_2_image     = [ 735 ; 325 ]  ; %; degtorad(camera_2_angle_yaw)]  ;
camera_position_2_world     = image_to_world__cob * camera_position_2_image  ;
camera_position_2_cells     = camera_position_2_world.*cells_per_metre  ;
camera_angle_2_rot2         = rotz(degtorad(camera_2_angle_yaw__screen))  ;     camera_angle_2_rot2=camera_angle_2_rot2(1:2,1:2)  ;
camera_optical_axis_direction_unit_2 = camera_angle_2_rot2*x_unit_screen__screen  ;
FoV_angle_2                         = 62;
[distance_uncertainty_costs_2, check_dist_map_2, check_angle_map_2] ...
    = sensor_model_func(scale_up_, world_x_extent_cells, world_y_extent_cells, camera_optical_axis_direction_unit_2, camera_position_2_cells , FoV_angle_2)  ;
distance_uncertainty_costs_2 = distance_uncertainty_costs_2'  ;
%-----  Third camera 
camera_3_angle_yaw__screen  = 180  ;
camera_position_3_image          = [ 825 ; 385 ]  ;  % ; degtorad(camera_3_angle_yaw)]  ;
camera_position_3_world           = image_to_world__cob * camera_position_3_image  ;
camera_position_3_cells             = camera_position_3_world.*cells_per_metre  ;
camera_angle_3_rot2                  = rotz(degtorad(camera_3_angle_yaw__screen))  ;     camera_angle_3_rot2 = camera_angle_3_rot2(1:2,1:2)  ;
camera_optical_axis_direction_unit_3 = camera_angle_3_rot2*x_unit_screen__screen  ;   % camera_optical_axis_direction_unit_1  ;
FoV_angle_3                                 = 62;
[distance_uncertainty_costs_3, check_dist_map_3, check_angle_map_3] ...
    = sensor_model_func(scale_up_, world_x_extent_cells, world_y_extent_cells, camera_optical_axis_direction_unit_3, camera_position_3_cells , FoV_angle_3)  ;
distance_uncertainty_costs_3 = distance_uncertainty_costs_3'  ;

cameras(1,:)= round(camera_position_1_cells);
cameras(2,:)=round( camera_position_2_cells );
cameras(3,:)=round(camera_position_3_cells);




eyeball = false
if eyeball
        % visual check
        f_distance_uncertainty_costs=figure;  s=surf(distance_uncertainty_costs_1);  s.EdgeColor='None';   xlabel('x'); ylabel('y'); zlabel('uncertainty');  % axis equal; 
        xlim([0 world_x_extent_cells(2)*scale_up]); ylim([0 world_y_extent_cells(2)*scale_up]); zlim([-0.1 1.1]);
        f_distance_uncertainty_costs.Name='distance_uncertainty_costs' ;
        % visual check
        f_check_angle_map = figure;  s=surf(check_angle_map );  s.EdgeColor='None';   xlabel('x'); ylabel('y'); zlabel('check_angle_map');  % axis equal; 
        f_check_angle_map.Name='f_check_angle_map ' ;
        % visual check
        f_check_dist_map = figure;  s=surf(check_dist_map);  s.EdgeColor='None';   xlabel('x'); ylabel('y'); zlabel('check_dist_map');  % axis equal; 
        f_check_dist_map.Name='check_dist_map'  ;
        % visual check
        f_distance_uncertainty_costs=figure;  s=surf(distance_uncertainty_costs_2);  s.EdgeColor='None'; xlabel('x'); ylabel('y'); zlabel('uncertainty');  % axis equal; 
        xlim([0 world_x_extent_cells(2)*scale_up]); ylim([0 world_y_extent_cells(2)*scale_up]); zlim([-0.1 1.1]);
        f_distance_uncertainty_costs.Name='distance_uncertainty_costs_2' ;
        % visual check
        f_check_angle_map = figure;  s=surf(check_angle_map_2 );  s.EdgeColor='None'; xlabel('x'); ylabel('y'); zlabel('check_angle_map');  % axis equal; 
        f_check_angle_map.Name='f_check_angle_map_2' ;        
        
        hold on ;  plot2_rows([ camera_position_1_cells camera_position_2_cells camera_position_3_cells ].*100  , 'rx' )
end


%% (2.1)
%----- Combined cost function v4:  as information gain dependent on error summation
%
%  order the payoffs by magnitude, use the first, then use the angular of the second to the first, then use the worst of third-to-second and third-to-first


distance_uncertainty_costs_stacked=[] ;
distance_uncertainty_costs_stacked(1,:,:) = 0.99*ones(size(distance_uncertainty_costs_1)) ;

distance_uncertainty_costs_stacked(2,:,:) = distance_uncertainty_costs_1 ;
distance_uncertainty_costs_stacked(3,:,:) = distance_uncertainty_costs_2  ;
distance_uncertainty_costs_stacked(4,:,:) = distance_uncertainty_costs_3  ;  % size(distance_uncertainty_costs_3)

size( distance_uncertainty_costs_stacked )
[payoff_order_from_distance_uncertainty_costs_stacked, index_order_of_sort] = sort( distance_uncertainty_costs_stacked, 1, 'ascend' )  ;

size( payoff_order_from_distance_uncertainty_costs_stacked )
size( index_order_of_sort ) 
payoff_order_from_distance_uncertainty_costs_stacked(:,300,300)
index_order_of_sort(:,257,178)

payoffs_map = zeros(size(distance_uncertainty_costs_stacked,2),size(distance_uncertainty_costs_stacked,3))  ;
payoffs_map_is_layer_1 = payoffs_map  ;

% check: the map outside the FoV is using the fixed layer=1=non-camera
bits_with_1_as_first =  index_order_of_sort(1,:,:)  ==1 ; size(bits_with_1_as_first);  class(bits_with_1_as_first)
f_bits_with_1_as_first=figure; idisp(squeeze(bits_with_1_as_first),'ynormal')  ;  f_bits_with_1_as_first.Name='cells outside a field of view, by assignment to the default uncertainty/cost';

%  
%( 2.2)  To combine two+ fields of view, handle this 'cost' as uncertainty at each point, as information gain at each point
%  
% 1) set up cameras, above in two sections
% 2) calculate the information gain above and here 
% 3) run simulation in the 'reduced' section  "(3 - reduced)"
%  Results:  need to show the difference between the two cost fields as a surf,  and the effecs on planned paths and allocations.
%{
    Here we treat information gain as observation from divergent angles to reduce the entropy of 
    the individual observations that is due to the error in the sensor model:  the errors correlate 
    with the direction of the target from the smart camera, so the information gain from each 
    observation/camera depends on the difference in angle to all other observations, as well as
    the distance to the target inherent in the individual sensor model.
%}
% num_layers=size(index_order_of_sort,1)  ;
% for xx_ = world_x_extent_cells(1):1:world_x_extent_cells(2)*scale_up        %  1 : 1216
%     for yy_ = world_y_extent_cells(1):1:world_y_extent_cells(2)*scale_up    %  1 :  803
%
index_order_of_sort_2 = permute( index_order_of_sort , [ 1 3 2 ] )  ;
here = 0 ;

for xx_ = world_x_extent_cells(1):1:world_x_extent_cells(2)*scale_up        %  1 : 1216
    for yy_ = world_y_extent_cells(1):1:world_y_extent_cells(2)*scale_up    %  1 :  803

here = 0 ;        
        % if 1 ==  index_order_of_sort(1 , xx_ , yy_)                                                 %   [  4         803        1216  ]    =   size(index_order_of_sort)  :-  image x = array columns 1216  : image y = array rows = 803  
        if 1 ==  index_order_of_sort(1 , yy_ , xx_)                                                 %   [  4         803        1216  ]    =   size(index_order_of_sort)  :-  image x = array columns 1216  : image y = array rows = 803  
            payoffs_map(yy_, xx_,size(cameras,1)+1) = 0;
            payoffs_map_is_layer_1( yy_ , xx_ ) = 1;
            continue;
        end
        camera_nums = index_order_of_sort(: , yy_ , xx_)  ;            
        camera_nums = camera_nums(camera_nums>1) ;
        payoff(1) = 1 * (1-distance_uncertainty_costs_stacked(camera_nums(1) , yy_ , xx_) ) ;
        
        
here = 1 ;
        target=[xx_,yy_]  ;
        camera_1_vec = target - cameras(camera_nums(1)-1,:)  ;
        camera_1_vec = normalise_vector(camera_1_vec)  ;  %  - take this as the base ,  FOR NOW
        camera_2_vec = target - cameras(camera_nums(2)-1,:)  ;
            camera_2_vec = normalise_vector(camera_2_vec)  ;
        camera_3_vec = target - cameras(camera_nums(3)-1,:)  ; 
            camera_3_vec = normalise_vector(camera_3_vec)  ;

here = 2 ;
    angle_diff = ...
       atan2(camera_1_vec(2),camera_1_vec(1)) ...
       - atan2(camera_2_vec(2),camera_2_vec(1))   ;
    angle_diff = radtodeg(angle_diff)  ;
    if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
    if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
    angle_diff_1_2 = abs( angle_diff ) ;

here = 3 ;
    angle_diff = ...
       atan2(camera_1_vec(2),camera_1_vec(1)) ...
       - atan2(camera_3_vec(2),camera_3_vec(1))   ;
    angle_diff = radtodeg(angle_diff)  ;
    if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
    if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
    angle_diff_1_3 = abs( angle_diff ) ;

here = 4 ;
    angle_diff = ...
       atan2(camera_2_vec(2),camera_2_vec(1)) ...
       - atan2(camera_3_vec(2),camera_3_vec(1))   ;
    angle_diff = radtodeg(angle_diff)  ;
    if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
    if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
    angle_diff_2_3 = abs( angle_diff )  ;            
        
here = 5 ;
        payoff(2) = abs(sin( angle_diff_1_2 )) * (1-distance_uncertainty_costs_stacked(camera_nums(2) , yy_, xx_) ) ;
        
here = 6 ;
        payoff(3) = max(abs(sin(angle_diff_1_3)),abs(sin(angle_diff_2_3)))* (1-distance_uncertainty_costs_stacked(camera_nums(3) , yy_, xx_) ) ;
        
here = 7 ;
        payoffs_map(yy_,xx_,:)=[payoff sum(payoff)]  ;
        %         payoffs_map(xx_,yy_)=sum(payoff);
    end
end

f_payoffs_map = figure; s=surf(payoffs_map(:,:,4)); s.EdgeColor='None';   f_payoffs_map.Name='payoffs_map';
f_payoffs_map_is_layer_1 = figure; s=surf(payoffs_map_is_layer_1); s.EdgeColor='None';  f_payoffs_map_is_layer_1.Name='payoffs_map_is_layer_1';
f_payoffs_map_is_layer_1 = figure; s=surf(flip(payoffs_map_is_layer_1,1)); s.EdgeColor='None';  f_payoffs_map_is_layer_1.Name='payoffs_map_is_layer_1';
    
%% AStar planning on the composite cost field
% Pick two points, do A* planning, make sure that it shows the effects of the shape of the information gain.
% See e.g. /mnt/nixbig/ownCloud/project_code/plan_to_observe/plan_to_observe_2b_AStar_line_traj_mv_if_conf.m
size(payoffs_map(:,:,4))
max(max(payoffs_map(:,:,4)))
costs_map = max(max(payoffs_map(:,:,4))) - payoffs_map(:,:,4) ;

costs_map(costs_map == max(max(payoffs_map(:,:,4)))) = max(max(payoffs_map(:,:,4)))*1000000;
costs_map = squeeze(costs_map ) ;

start_1  = [265 ; 262]  ;
goal_1 = [144 ; 218]  ;
%goal_1 = [144 ; 262]  ;
map_1 = costs_map ;

tic
            [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
            [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
toc            
            
f_costs_map = figure; 
s_costs_map = surf(map_1); 
s_costs_map.EdgeColor = 'none';
%  s_costs_map.EdgeColor = [0 0 0];
f_costs_map.Name='costs_map';
hold on; plot3_rows([start_1 ; 2.1], 'rx', 'LineWidth',2) 
hold on; plot3_rows([goal_1 ; 2.1], 'bd', 'LineWidth',2) 
size(total_path_start_to_goal__)
plot3_rows( [ total_path_start_to_goal__ ; (max(max(map_1))*1.01)*ones(1,size(total_path_start_to_goal__,2))] , 'm:', 'LineWidth',2)


axes_current = f_costs_map.CurrentAxes()
axes_current_cam_posn = axes_current.CameraPosition
axes_current_cam_target = axes_current.CameraTarget
axes_current.CameraUpVector
axes_current.CameraViewAngle

%  plot the cost of traversal 
figure;   hold on; grid on;
points_ = zeros(1,size(total_path_goal_to_start__,2));
for ii_ = 1:size(total_path_goal_to_start__,2)
    points_(ii_) =  map_1(total_path_goal_to_start__(2,ii_),total_path_goal_to_start__(1,ii_))  ;
end
plot( points_ , 'r' )
plot( [1,size(total_path_goal_to_start__,2)] , [ max(max(map_1)) , max(max(map_1)) ] )
xlim( [ 1 size(total_path_goal_to_start__,2) ] )
ylim( [ 0 max(max(map_1))*1.1] )

%%


%% for DStarMoo 
addpath('/mnt/nixbig/ownCloud/project_code/3rd_party/robotics-toolbox-matlab/')
addpath('/mnt/nixbig/ownCloud/project_code/')
%%  (3 - large)   DStarMoo planning on separate cost fields with multi-objective (not Pareto) 
%{
See   /mnt/nixbig/ownCloud/project_code/temp__AStar_DStar__Moo_PO__corridor.m   -  section "multi-objective and dynamic objective DStar"
%}

f_payoffs_map = figure; 
s=surf(payoffs_map(:,:,1)); zlim([ 0 max(max(payoffs_map(:,:,4)))] ) % payoffs 1-3 are per-camera, #4 is the composite, not to be used here
s.EdgeColor='none'
hold on
s=surf(payoffs_map(:,:,4));
s.EdgeColor='none'
f_payoffs_map.Name='payoffs_map';     

        figure_named('plan and move')

        % payoffs_map is the map of projected information gain from VOS smart cameras: it belongs to VOS
        payoffs_map;

        % map_1 is the base map passed to Navigation; remains zeros to avoid problems between Navigation and DStarMoo:  belongs to DStarMoo
        map_1 = zeros(size(payoffs_map,1),size(payoffs_map,2)) ;  % no walls

        % belong to the robot
        start_1  = [265 ; 262]  ;
        goal_1 = [144 ; 218]  ;
        
        
        % flooplan_ is the flooplan: it is the world model of static obstacles:  it belongs to the map owner
        floorplan_ = zeros(size(map_1));
        floorplan_(215:260,209:211) = 1000000 ;  % wall to mostly bisect the middle FoV
        floorplan_(215:260,249:251) = 1000000 ;  % wall 
        floorplan_(260:260,209:251) = 1000000 ;  % wall  
        
        wall__reduced_image = [     195, 95 ;
                                        195, 85 ]  ;
        wall__data = wall__reduced_image.*scaledown  ;
        floorplan_( wall__data(1,2):wall__data(2,2),wall__data(1,1)-1 : wall__data(2,1)+1) = 1000000 ;  
                                    
        
        % inflated_flooplan is the flooplan inflated for safe robot navigation: it belongs to the robot: 
        inflated_flooplan = ismooth(floorplan_,2) ;
        
        as = DstarMOO(map_1,inflated_flooplan);    % INFLATION  create Navigation object
        
        costs_map = max(max(payoffs_map(:,:,1))) - payoffs_map(:,:,1) ;
        costs_map = squeeze(costs_map ) ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
        as.addCost(1,normA);        % add 1st add'l cost layer L
        
        costs_map = max(max(payoffs_map(:,:,2))) - payoffs_map(:,:,2) ;
        costs_map = squeeze(costs_map ) ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
        as.addCost(2,normA);        % add 1st add'l cost layer L
        
        costs_map = max(max(payoffs_map(:,:,3))) - payoffs_map(:,:,3) ;
        costs_map = squeeze(costs_map ) ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
%         normA = zeros(size(normA )) ;
        as.addCost(3,normA);        % add 1st add'l cost layer L
        
         tic
         as.plan(goal_1,5,start_1);       % setup costmap for specified goal ;  N = number of cost layers to use, where 1=distance and 2=heuristic
         toc
         %figure; pause(1); as.path(start_1);        % plan solution path star-goal, animate
         P = as.path(start_1);    % plan solution path star-goal, return path 
         size(P)
         path_plotpoints = [ P 1.1*ones(size(P,1),1)]  ;
         
        figure(f_payoffs_map);  pause(1); hold on;                %plot3_rows( [ P 1.1*ones(size(P,1),1)]' , 'rx', 'LineWidth',2)  % plot flat 
        for ii_ = 1:size(P,1)
            plot3( P(ii_,1),P(ii_,2),  payoffs_map(P(ii_,2),P(ii_,1),4)+0.01, 'rs', 'LineWidth',2)  % plot across the surface
        end

        
                 V = [209 215 2; 211 215 2; 211 260 2; 209 260 2];
                 F = [1 2 3 4];
                 p___ = patch('Faces',F,'Vertices',V)
        
                 V = [249 215 2; 251 215 2; 251 260 2; 249 260 2];
                 F = [1 2 3 4];
                 p___ = patch('Faces',F,'Vertices',V)
        
        
                x1=260;x2=260;
                y1=209;y2=251;
                 V = [ y1  x1  2; y2  x1  2; y2 x2  2; y1 x2  2];
                 F = [1 2 3 4];
                 p___ = patch('Faces',F,'Vertices',V)
                 
                x1=wall__data(1,2);x2=wall__data(2,2);
                y1=wall__data(1,1);y2=wall__data(2,1);
                h=1.071191411753806;
                h=2 ;
                 V = [ y1  x1  h; y2  x1  h; y2 x2  h; y1 x2  h];
                 V = V*[1/scaledown 0 0; 0 1/scaledown 0 ; 0 0 1]  ;
                 F = [1 2 3 4];
                 p___ = patch('Faces',F,'Vertices',V)
                 
                 

        %{ 
        Sensor_model_as_object(              floorplan_,             
                    payoffs_map_,             start_, goal_,             robot_size_)           
        %}
        size(payoffs_map)
        payoffs_map_costmap_layers = payoffs_map(:,:,1:3)  ;
        s1111 = Sensor_model_as_object('s1111',floorplan_,payoffs_map_costmap_layers, start_1,goal_1,  1  ) ;        
        path = s1111.planningStep()  ;
        size(path)
        path(1,:)
        path(end,:)
        robots{1}.status = s1111.moveAStep
        robots{1}.location = s1111.robot_location        
        
        s2222 = Sensor_model_as_object('s2222',floorplan_,payoffs_map_costmap_layers, start_1-[10;10],goal_1+[10;10],  1  ) ;        
        path = s2222.planningStep()  ;
        size(path)
        path(1,:)
        path(end,:)
        robots{2}.status = s2222.moveAStep
        robots{2}.location = s2222.robot_location
        
        robot_in_play = true
        while robot_in_play
            
            if  ~isequal( robots{1}.status , Sensor_model_as_object.AT_GOAL )
                robots{1}.status = s1111.moveAStep                
                robots{1}.location = s1111.robot_location
            end
            if  ~isequal( robots{2}.status , Sensor_model_as_object.AT_GOAL )
                robots{2}.status = s2222.moveAStep
                robots{2}.location = s2222.robot_location            
            end
            
            robot_in_play_now = false;
            for ii_ = 1:size(robots,2)
                if ~isequal( robots{ii_}.status , Sensor_model_as_object.AT_GOAL )
                    robot_in_play_now = true ;
                    break  ;
                end
            end
            robot_in_play = robot_in_play_now;            
        end
        
         %%  (3 - reduced) reduced map size

         scaledown = 4;


base_size = [size(payoffs_map,1),size(payoffs_map,2)]  ;

map_1 = zeros(base_size) ;  % no walls
map_1=map_1(1:scaledown:end,1:scaledown:end) ;

payoffs_map_reduced=payoffs_map(1:scaledown:end,1:scaledown:end,:)  ; 

        
        %   start_1  = [265 ; 262]  ;
        start_1  = [73  ;  33].*cells_per_metre  ;
        start_1  = round(start_1 ./ scaledown) ;
        %   goal_1 = [144 ; 218]  ;
        goal_1 = [83  ;  39].*cells_per_metre  ;
        goal_1 = round(goal_1 ./ scaledown) ;
        
        
        % flooplan_ is the flooplan: it is the world model of static obstacles:  it belongs to the map owner
        floorplan_ = zeros(base_size);        
        
        floorplan_=floorplan_(1:scaledown:end,1:scaledown:end) ;  
        size(floorplan_)
        floorplan_(round([215:260]./ scaledown),round([209:211]./ scaledown)) = 1000000 ;  % wall to mostly bisect the middle FoV
        floorplan_(round([215:260]./ scaledown),round([249:251]./ scaledown)) = 1000000 ;  % wall 
        %   floorplan_(rows,cols)
        floorplan_( round([209:251]./ scaledown) , round([260:260]./ scaledown )) = 1000000 ;  % wall
        floorplan_( round([260:260]./ scaledown) , round([209:251]./ scaledown )) = 1000000 ;  % wall
        
        wall__reduced_image = [    195, 85 ;
                                                        195, 95 ]  ;
        wall__data_full = wall__reduced_image.*scaledown  ;        
        wall__data_reduced = round(wall__data_full ./ scaledown)  ;
        floorplan_( wall__data_reduced(1,2):wall__data_reduced(2,2),wall__data_reduced(1,1)-1 : wall__data_reduced(2,1)+1) = 1000000 ;  
        
        
        % as = DstarMOO(map_1,floorplan_);    % create Navigation object        
        
        %as = DstarMOO(map_1, (floorplan>0).*100000000000 );    % create Navigation object
        
        floorplan_2 = zeros(size(floorplan))  ;
        floorplan_2(floorplan<1) = inf  ;        
        floorplan_2 = floorplan_2( 1:scaledown:end , 1:scaledown:end )  ;
        as = DstarMOO(map_1, floorplan_2 );    % create Navigation object
        
        
        
        costs_map = max(max(payoffs_map(:,:,1))) - payoffs_map(:,:,1) ;
        costs_map = squeeze(costs_map ) ;
        costs_map(floorplan<1) = 10*max(max(payoffs_map(:,:,1))) ;
        costs_map = costs_map(1:scaledown:end,1:scaledown:end) ;
        % payoffs_map_reduced(:,:,1) = costs_map  ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
        as.addCost(1,normA);        % add 1st add'l cost layer L
        
        costs_map = max(max(payoffs_map(:,:,2))) - payoffs_map(:,:,2) ;
        costs_map = squeeze(costs_map ) ;
        costs_map(floorplan<1) = 10*max(max(payoffs_map(:,:,2))) ;
        costs_map = costs_map(1:scaledown:end,1:scaledown:end) ;
        % payoffs_map_reduced(:,:,2) = costs_map  ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
        as.addCost(2,normA);        % add 1st add'l cost layer L
        
        costs_map = max(max(payoffs_map(:,:,3))) - payoffs_map(:,:,3) ;
        costs_map = squeeze(costs_map ) ;
        costs_map(floorplan<1) = 10*max(max(payoffs_map(:,:,3))) ;
        costs_map = costs_map(1:scaledown:end,1:scaledown:end) ;
        % payoffs_map_reduced(:,:,3) = costs_map  ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
%         normA = zeros(size(normA )) ;
        as.addCost(3,normA);        % add 1st add'l cost layer L
        
         
         
         
         
f_payoffs_map = figure; 
s=surf(payoffs_map(1:scaledown:end,1:scaledown:end,1)); zlim([ 0 max(max(payoffs_map(:,:,4)))] ) % payoffs 1-3 are per-camera, #4 is the composite, not to be used here
s.EdgeColor='none'
hold on
s=surf(payoffs_map(1:scaledown:end,1:scaledown:end,4));
s.EdgeColor='none'
f_payoffs_map.Name='payoffs_map';     
%-- draw the walls
Vz = max(max(payoffs_map(:,:,4))) ;
 V = [209 215 Vz; 211 215 Vz; 211 260 Vz; 209 260 Vz];
 V = V*[1/scaledown 0 0; 0 1/scaledown 0 ; 0 0 1]    ;
 F = [1 2 3 4];
 p___ = patch('Faces',F,'Vertices',V)        ;
 V = [249 215 Vz; 251 215 Vz; 251 260 Vz; 249 260 Vz];
 V = V*[1/scaledown 0 0; 0 1/scaledown 0 ; 0 0 1]    ;
 F = [1 2 3 4];
 p___ = patch('Faces',F,'Vertices',V)       ;
x1=260;x2=260;
y1=209;y2=251;
 V = [ y1  x1  Vz; y2  x1  Vz; y2 x2  Vz; y1 x2  Vz];
 V = V*[1/scaledown 0 0; 0 1/scaledown 0 ; 0 0 1]  ;
 F = [1 2 3 4];
 p___ = patch('Faces',F,'Vertices',V)       ;
 
%x1 = wall__reduced_image(1,1);  x2 = wall__reduced_image(2,1)  ;  y1 = wall__reduced_image(1,2)  ;  y2 = wall__reduced_image(2,2) ;
x1 = wall__data_full(1,1);  x2 = wall__data_full(2,1)  ;  y1 = wall__data_full(1,2)  ;  y2 = wall__data_full(2,2)  ;
[ x1 y1 ; x2 y2 ]
%   size(floorplan_)  =  201   781  
floorplan_( y1:y2 , x1-1:x2+1 ) = 1000000 ;  
 V = [      x1 y1  Vz   ;   x1 y2   Vz     ;       x2 y2  Vz  ;   x2 y1  Vz  ];
 V = V*[1/scaledown 0 0; 0 1/scaledown 0 ; 0 0 1]  ;
 F = [1 2 3 4];
 p___ = patch('Faces',F,'Vertices',V)       ;
 
            size(floorplan)
            size(payoffs_map) 
            %  floorplan_display_as_surf = (1-floorplan).*Vz*0.75  ;
            %  floorplan_display_as_surf = (1-floorplan).*Vz  ; 
            % s=surf(  floorplan_display_as_surf(1:scaledown:end,1:scaledown:end,1)  ); 
            % s.EdgeColor='None'
            % s.FaceColor='k'
 
%-- demo plan BETTER : use the same object as the robots : fixed goal, fixed starting point : test the time to plan
        payoffs_map_costmap_layers = payoffs_map_reduced(:,:,1:3)  ;  % CHANGE THIS TO CHANGE THE ROBOT PLANNING
        floorplan_  %  MAKE SURE THAT THIS IS RIGHT
        demo_start_location %  MAKE SURE THAT THIS IS RIGHT
        demo_goal_location %  MAKE SURE THAT THIS IS RIGHT
        s_demo_plan = Sensor_model_as_object('s_demo_plan', floorplan_ , payoffs_map_costmap_layers , demo_start_location , demo_goal_location ,  1  ) ;        


%-- demo plan : fixed goal, fixed starting point : test the time to plan
figure_named('demo plan : fixed goal, fixed starting point')                          
        figure(f_payoffs_map);  pause(1); hold on;                %plot3_rows( [ P 1.1*ones(size(P,1),1)]' , 'rx', 'LineWidth',2)  % plot flat         
         tic
         as.plan(goal_1,scaledown,start_1);       % setup costmap for specified goal ;  N = number of cost layers to use, where 1=distance and 2=heuristic
         toc
         %figure; pause(1); as.path(start_1);        % plan solution path star-goal, animate
         P = as.path(start_1);    % plan solution path star-goal, return path 
         size(P)
         path_plotpoints = [ P 1.1*ones(size(P,1),1)]  ;
        for ii_ = 1:size(P,1)
            plot3( P(ii_,1),P(ii_,2),  payoffs_map_reduced(P(ii_,2),P(ii_,1),4)+0.01, 'rs', 'LineWidth',1)  % plot across the surface
        end
 
%-- draw the walls


        
        %  !! problem with wall in base map
        %         V = [209 215 2; 211 215 2; 211 260 2; 209 260 2];
        %         %  V = [215 209 2; 215 211 2; 260 211 2; 260 209 2];
        %         F = [1 2 3 4];
        %         p___ = patch('Faces',F,'Vertices',V)
        
        f_rendezvous_map = figure_named('Rendezvous','default');
        hold on
%         s_f_rendezvous_map=surf(zeros(size(payoffs_map(1:scaledown:end,1:scaledown:end,4))));
%         s_f_rendezvous_map.EdgeColor='none';
        
        size(payoffs_map_reduced)
        payoffs_map_costmap_layers = payoffs_map_reduced(:,:,1:3)  ;  % CHANGE THIS TO CHANGE THE ROBOT PLANNING
        
        s1111 = Sensor_model_as_object('s1111',floorplan_,payoffs_map_costmap_layers, start_1,goal_1,  1  ) ;        
        path = s1111.planningStep()  ;
        size(path)
        path(1,:)
        path(end,:)
        robots{1}.robot_sim = s1111  ;
        robots{1}.status = s1111.moveAStep  ;
        robots{1}.location = s1111.robot_location  ;
        robots{1}.location_hist = s1111.robot_location  ;
        
        s2222 = Sensor_model_as_object('s2222',floorplan_,payoffs_map_costmap_layers, start_1-[10;10],goal_1+[10;10],  1  ) ;        
        path = s2222.planningStep()  ;
        size(path)
        path(1,:)
        path(end,:)
        robots{2}.robot_sim = s2222  ;
        robots{2}.status = s2222.moveAStep  ;
        robots{2}.location = s2222.robot_location  ;
        robots{2}.location_hist = s2222.robot_location  ;
        
        %  pause
        
        robot_in_play = true  ;
        while robot_in_play
            
            if  ~isequal( robots{1}.status , Sensor_model_as_object.AT_GOAL )
                robots{1}.status = s1111.moveAStep  ;
                robots{1}.location = s1111.robot_location  ;
                robots{1}.location_hist(end+1,:) = s1111.robot_location  ;
            end
            if  ~isequal( robots{2}.status , Sensor_model_as_object.AT_GOAL )
                robots{2}.status = s2222.moveAStep  ;
                robots{2}.location = s2222.robot_location  ;
                robots{2}.location_hist(end+1,:) = s2222.robot_location  ;
            end
            
            robots{1}.robot_sim.changeGoal(robots{2}.location)  ;            
            robots{2}.robot_sim.changeGoal(robots{1}.location)  ;
            
%             path_plots{1}.last_plot = [];
%             path_plots{1}.hist = [];
%             path_plots{2}.last_plot = [];
%             path_plots{2}.hist = [];
            
            robot_in_play_now = false;
            for ii_ = 1:size(robots,2)
                if ~isequal( robots{ii_}.status , Sensor_model_as_object.AT_GOAL )
                    robot_in_play_now = true ;
                    break  ;
                end
            end
            robot_in_play = robot_in_play_now;            
            
            figure(f_rendezvous_map);  
            cla
            hold on; grid on; axis equal;
            xlabel('x'); ylabel('y'); zlabel('z');
            xlim([0 size(payoffs_map_reduced,1)]) ; ylim([0 size(payoffs_map_reduced,2)]) ;
            for ii_ = 1:size(robots,2)
                if ~isequal( robots{ii_}.status , Sensor_model_as_object.AT_GOAL )                    
                    figure(f_rendezvous_map);  hold on;                %plot3_rows( [ P 1.1*ones(size(P,1),1)]' , 'rx', 'LineWidth',2)  % plot flat 
%                     if ~isempty(path_plots{ii_}.last_plot) ; display('delete(path_plots{ii_})'); delete(path_plots{ii_}.last_plot)  ; end;
                    if 1==ii_
                        %plot3( robots{ii_}.location(1),robots{ii_}.location(2),  payoffs_map_reduced(robots{ii_}.location(2),robots{ii_}.location(1),4)+0.01, 'k', 'LineWidth',1)  % plot across the surface
                        plot( robots{ii_}.location(1),robots{ii_}.location(2), 'ks', 'LineWidth',1)  % plot across the surface
                        plot( robots{ii_}.robot_sim.current_path(:,1)',robots{ii_}.robot_sim.current_path(:,2)', 'k:', 'LineWidth',1) ;  % plot across the surface
%                         
%                         path_plots{ii_}.last_plot=plot( robots{ii_}.robot_sim.current_path(:,1)',robots{ii_}.robot_sim.current_path(:,2)', 'k:', 'LineWidth',1) ;  % plot across the surface
%                         path_plots{ii_}.current = path_plots{ii_}.last_plot  ;
                    else
                        %plot3( robots{ii_}.location(1),robots{ii_}.location(2),  payoffs_map_reduced(robots{ii_}.location(2),robots{ii_}.location(1),4)+0.01, 'b', 'LineWidth',1)  % plot across the surface
                        plot( robots{ii_}.location(1),robots{ii_}.location(2), 'bd', 'LineWidth',1)  % plot across the surface
                        plot( robots{ii_}.robot_sim.current_path(:,1)',robots{ii_}.robot_sim.current_path(:,2)', 'b:', 'LineWidth',1) ;  % plot across the surface
%                         
%                         path_plots{ii_}.last_plot=plot( robots{ii_}.robot_sim.current_path(:,1)',robots{ii_}.robot_sim.current_path(:,2)', 'b:', 'LineWidth',1) ;  % plot across the surface
%                         path_plots{ii_}.current = path_plots{ii_}.last_plot  ;
                    end
                else
                    figure(f_rendezvous_map);  hold on;                %plot3_rows( [ P 1.1*ones(size(P,1),1)]' , 'rx', 'LineWidth',2)  % plot flat 
                    %plot3( robots{ii_}.location(1),robots{ii_}.location(2),  payoffs_map_reduced(robots{ii_}.location(2),robots{ii_}.location(1),4)+0.01, 'g', 'LineWidth',1)  % plot across the surface
                    plot( robots{ii_}.location(1),robots{ii_}.location(2), 'gx', 'LineWidth',3)  % plot across the surface
                end
                    drawnow;  pause(0.1) ;                     
            end            
            
            drawnow;
        end
       
         
%%  To combine two+ fields of view, handle this 'cost' as uncertainty at each point:
%{
variance_ = ( g1_variance * g2_variance ) / (g1_variance + g2_variance) ; 
standard_deviation_ = sqrt(variance_)

Game: Try: rotate the sigma/covariance into the world coordinates as rays diverge from optical axis.
 grid cell --> 
    1)  ray angle to camera position -->  gaussian rotation  --> rotation of covariance matrix
    2)  distance from camera --> gaussian magnitude 

    Note: no covariance -> axis-aligned distribution :  which is TRUE but the axes are polar coordinates of the cameras.
        -> covariance in world grid by rotating the covariance matrix from diagonal matrix. :  NOPE - mvnpdf wants a diagonal
    Try:  treat the axes independently: they're supposed to be, after all.
        https://au.mathworks.com/help/stats/multivariate-normal-distribution.html


    NOTE:  I _can_ precompute these if I need to, even if it is slow.

Game: trying to find a good cost function for AStar
    Just take the lowest uncertainty across all the fields of view.
    See if it works for getting a reasonable path  -->  getting some results  .
NOTE:  I can justify this:  can only combine the observations from both cameras to give reduced uncertainty if both observe the robot at the same position/time
--> synchronous observations
    Relax that --> just use the best uncertainty.
    --> treat as a vector:  
        orientation_of_ray_at_cell* ( [1;0].*uncertainty_x(dist) )
        orientation_of_ray_at_cell* ( [0;1].*uncertainty_y(dist) )



See https://ccrma.stanford.edu/~jos/sasp/Product_Two_Gaussian_PDFs.html 
    Treat as zero mean to start with, which is a big simplificaiton _if_ it's true.
    Then have 2D variance at each point in the FoV 
    Take angular error magnitude as 1/3 linear error magnitude.
    Take linear error magnitude as the uncertainty value from above 

See /mnt/nixbig/ownCloud/project_code/20181008/gaussian_mult_example.m
%}


%%  From the above, for cameras 1 to i, take uncertainty_cost as min(uncertainty_i)
% Start with 2 cameras




%% 
% this ??
radtodeg(atan(camera_optical_axis_direction_unit_1(2)/camera_optical_axis_direction_unit_1(1)))











                %% check layout
                x_unit_generic = [1;0]  ;
                y_unit = [0;1]  ;
                x_unit_origin = [camera_position_1_cells(1:2) x_unit_generic+camera_position_1_cells(1:2) ]  ;
                y_unit_origin = [camera_position_1_cells(1:2) y_unit+camera_position_1_cells(1:2) ]  ;
                x_unit_rotated = [camera_position_1_cells(1:2) camera_angle_1_rot2(1:2,1:2)*x_unit_generic+camera_position_1_cells(1:2) ]  ;
                y_unit_rotated = [camera_position_1_cells(1:2) camera_angle_1_rot2(1:2,1:2)*y_unit+camera_position_1_cells(1:2) ]  ;

                camera_optical_axis_direction_unit_1 = camera_angle_1_rot2(1:2,1:2)*x_unit_generic  ;
                camera_optical_axis_2units = [ camera_position_1_cells(1:2) camera_position_1_cells(1:2)+camera_optical_axis_direction_unit_1*2]  ;

                %  Draw the camera in the world

                f_camera_in_world = figure;  hold on; 
                plot2_rows_rotate_robot_to_diagram(  [0 world_x_extent_cells(2); 0 world_y_extent_cells(2)],'bo') ; 
                axis equal
                plot2_rows_rotate_robot_to_diagram( x_unit_origin , 'r')    %  Always good to remember that diagram directions aren't robot directions
                plot2_rows_rotate_robot_to_diagram( y_unit_origin , 'g')
                plot2_rows_rotate_robot_to_diagram( x_unit_rotated , 'r')
                plot2_rows_rotate_robot_to_diagram( y_unit_rotated , 'g')
                plot2_rows_rotate_robot_to_diagram( camera_optical_axis_2units , 'k')

                %  FoV: angle of view is 90 degrees (or 60 degrees, or whatever)
                FoV_angle_1 = 62 ;
                FoV_left_rot = rotz(degtorad(FoV_angle_1/2))  ;  FoV_left_rot = FoV_left_rot(1:2,1:2)  ;
                FoV_right_rot = rotz(degtorad(-FoV_angle_1/2))  ;  FoV_right_rot = FoV_right_rot(1:2,1:2)  ;  
                FoV_left_dir = FoV_left_rot*camera_optical_axis_direction_unit_1  ;
                FoV_right_dir = FoV_right_rot*camera_optical_axis_direction_unit_1  ;
                FoV_left_lim = [ camera_position_1_cells(1:2) FoV_left_dir+camera_position_1_cells(1:2) ]  ;
                FoV_right_lim = [ camera_position_1_cells(1:2) FoV_right_dir+camera_position_1_cells(1:2) ]  ;

                figure(f_camera_in_world)
                plot2_rows_rotate_robot_to_diagram( FoV_left_lim , 'c')  ;
                plot2_rows_rotate_robot_to_diagram( FoV_right_lim , 'm')  ;
                %   cla
                plot2_rows_rotate_robot_to_diagram(  [0 world_x_extent_cells(2); 0 world_y_extent_cells(2)],'bo')  ; 

                % plot2_rows_rotate_robot_to_diagram( [ 10 ; 30 ]  , 'rx' ) 
                % plot2_rows_rotate_robot_to_diagram( [ 12 15 ; 32 35 ]  , 'r' ) 

                %  redo it as homogeneous coordinates
                FoV_left_rot = rotz(degtorad(FoV_angle_1/2))  ; 
                FoV_left_dir = FoV_left_rot * e2h(camera_optical_axis_direction_unit_1)  ;
                camera_position_h = e2h(camera_position_1_cells(1:2))  ;
                FoV_left_lim = [ camera_position_h   FoV_left_dir+camera_position_h ]  ;
                plot2_rows_rotate_robot_to_diagram( FoV_left_lim , 'c')  ;

                
%%
%       Factored out as sensor_model_func.m

distance_uncertainty_costs_1 = zeros(  world_x_extent_cells(2)*scale_up,world_y_extent_cells(2)*scale_up  )  ;

% visual check
check_dist_map = zeros(  world_x_extent_cells(2)*scale_up,world_y_extent_cells(2)*scale_up  )  ;
% visual check
check_angle_map = zeros(  world_x_extent_cells(2)*scale_up,world_y_extent_cells(2)*scale_up  )  ;
for xx_ = world_x_extent_cells(1):1:world_x_extent_cells(2)*scale_up
    for yy_ = world_y_extent_cells(1):1:world_y_extent_cells(2)*scale_up
       % angle of 2 relative to 1= atan2(v2.y,v2.x) - atan2(v1.y,v1.x)  
        diff_vec = [xx_ ;yy_] - camera_position_1_cells(1:2).*scale_up  ;
        diff_vec_magnitude_ = norm_2(diff_vec,1)  ;
        unit_diff_vec = diff_vec ./ diff_vec_magnitude_  ;   
       angle_diff = ...
           atan2(unit_diff_vec(2),unit_diff_vec(1)) ...
           - atan2(camera_optical_axis_direction_unit_1(2),camera_optical_axis_direction_unit_1(1))   ;
        angle_diff = radtodeg(angle_diff)  ;
        if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
        if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
        
        dist = norm_2(  [xx_ ;yy_] - camera_position_1_cells(1:2).*scale_up , 1  )  ;
        
        check_dist_map(xx_,yy_) = dist;
        
        distance_uncertainty_cost= (0.714285714285714*0.0005*exp(dist/10))  ;
        if distance_uncertainty_cost > 1
            distance_uncertainty_cost = 1  ;
        end
        if abs( angle_diff ) > FoV_angle_1/2; distance_uncertainty_cost = 1;   end;
        distance_uncertainty_costs_1(xx_,yy_) = distance_uncertainty_cost;
        
        
        % visual check
        check_angle_map(xx_,yy_) = abs( angle_diff ) ; 
        if abs( angle_diff ) > FoV_angle_1/2; check_angle_map(xx_,yy_) = 0;   end;            
    end
end
                
%% 
%       Check the information gain.

% OK, but is the dissimilarty in sensor/algorithm/angle not important ; if I stack 2 of the 3 cameras together, is their field of view really better than the
% other?
%  3 cameras
world = zeros(101,101)  ;
target = [  51,51 ]  ;
camera_1 = [ 10, 51 ]  ;    % expect extra contribution between camera_1 and (camera_2 | camera_3)
cameras(1,:)= [ 10, 51 ]  ;
camera_2 = [ 51, 10 ]  ;    % expect extra contribution between camera_1 and (camera_2 | camera_3)  but  expect no extra contribution between camera_2 and camera_3
cameras(2,:)=camera_2;
camera_3 = [ 51, 100 ]  ;   %  expect extra contribution between camera_1 and (camera_2 | camera_3)  but  expect no extra contribution between camera_2 and camera_3
cameras(3,:)=camera_3;

camera_1_vec = target - camera_1  ;
    camera_1_vec = normalise_vector(camera_1_vec)  ;  %  - take this as the base ,  FOR NOW
camera_2_vec = target - camera_2  ;
    camera_2_vec = normalise_vector(camera_2_vec)  ;
camera_3_vec = target - camera_3  ; 
    camera_3_vec = normalise_vector(camera_3_vec)  ;

    
angle_diff = ...
   atan2(camera_1_vec(2),camera_1_vec(1)) ...
   - atan2(camera_2_vec(2),camera_2_vec(1))   ;
angle_diff = radtodeg(angle_diff)  ;
if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
abs( angle_diff )  
    
angle_diff = ...
   atan2(camera_1_vec(2),camera_1_vec(1)) ...
   - atan2(camera_3_vec(2),camera_3_vec(1))   ;
angle_diff = radtodeg(angle_diff)  ;
if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
abs( angle_diff )  
    
angle_diff = ...
   atan2(camera_2_vec(2),camera_2_vec(1)) ...
   - atan2(camera_3_vec(2),camera_3_vec(1))   ;
angle_diff = radtodeg(angle_diff)  ;
if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
angle_diff = abs( angle_diff )  
% QU:  question here is: is angle between 90 and 180 _different_ to an angle between 1 and 90 .- in this case NO
if angle_diff > 90 ; angle_diff = abs(angle_diff - 180)  ; end
% QU:  question here is: is there a threshold of utility e.g. 10 to 90 rather than 1 to 90 - in this case, no, but the advantage is low for small angles, maxing
% at 90
% approximate payoff as sin
sin(degtorad(angle_diff))


%%


%----- Combined cost function v1:  as maximum observability from any one camera = cost_c(x,y)= min(  cost_i(x,y) ) for i = 1 : num cameras 
            %----- NOTE : try this with DStarMoo with one camera per cost layer
distance_uncertainty_costs_stacked_v1 = cat(3, 0.99*ones(size(distance_uncertainty_costs_1)), distance_uncertainty_costs_1, distance_uncertainty_costs_2)  ;
[ distance_uncertainty_costs_combined , camera_per_cell ] = min(distance_uncertainty_costs_stacked_v1,[],3)  ;
f_camera_per_cell = figure; surf(squeeze(camera_per_cell) ); f_camera_per_cell.Name='camera_per_cell: 2 smart cameras'  ;


%----- Combined cost function v2:  as minimum 
distance_uncertainty_costs_stacked=[] ;
distance_uncertainty_costs_stacked(1,:,:) = 0.99*ones(size(distance_uncertainty_costs_1)) ;

distance_uncertainty_costs_stacked(2,:,:) = distance_uncertainty_costs_1 ;
distance_uncertainty_costs_stacked(3,:,:) = distance_uncertainty_costs_2  ;
distance_uncertainty_costs_stacked(4,:,:) = distance_uncertainty_costs_3  ;  % size(distance_uncertainty_costs_3)
[ distance_uncertainty_costs_combined , camera_per_cell ] = min(distance_uncertainty_costs_stacked,[],1)  ;  size(distance_uncertainty_costs_combined )
f_camera_per_cell_num2 = figure; surf(squeeze(camera_per_cell) ); f_camera_per_cell_num2.Name='camera_per_cell: 3 smart cameras'  ;

%----- Combined cost function v3:  straight sum of (1-uncertainty) :  assumes totally independent 
%   Now look at it as the expected information gain per unit time, with the cameras giving independent observations/pose estimates
%   At each point in more than one FoV, can expect more information gain per unit time.
%   Take information gain = entropy reduction = uncertainty reduction 
%   Invert the uncertainty, and sum 
distance_uncertainty_costs_stacked=[] ;
distance_uncertainty_costs_stacked(1,:,:) = 0.99*ones(size(distance_uncertainty_costs_1)) ;

distance_uncertainty_costs_stacked(2,:,:) = distance_uncertainty_costs_1 ;
distance_uncertainty_costs_stacked(3,:,:) = distance_uncertainty_costs_2  ;
distance_uncertainty_costs_stacked(4,:,:) = distance_uncertainty_costs_3  ;  % size(distance_uncertainty_costs_3)
info_gains_negcosts_stacked = zeros(size(distance_uncertainty_costs_stacked))  ;
for cam_num_ = 1:size(info_gains_negcosts_stacked,1)
    info_gains_negcosts_stacked(cam_num_,:,:) = 1-distance_uncertainty_costs_stacked(cam_num_,:,:)  ;
end
sum_info_gains_negcosts_stacked = sum(info_gains_negcosts_stacked,1)  ;
sum_info_gains_negcosts_stacked = squeeze(sum_info_gains_negcosts_stacked)  ;
f_sum_info_gains_negcosts_stacked=figure;  surf( sum_info_gains_negcosts_stacked ); f_sum_info_gains_negcosts_stacked.Name='sum_info_gains_negcosts_stacked';

