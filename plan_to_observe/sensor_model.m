%%  Sensor model : 
%       uncertainty vs. distance of feature to camera optical centre
%       uncertainty vs. angle of robot feature to camera optical axis
%%
addpath('/mnt/nixbig/ownCloud/project_code/plan_to_observe/')
%%

% Visualise the error/uncertainty model 
dist = 0.0 : 0.1 : 8-0.1
uncertainty_vs_distance = (0.714285714285714*0.0005*exp(dist))   
figure; 
subplot(1,2,1); hold on; grid on; xlabel('distance (0.1m)'); ylabel('uncertainty vs distance');
plot(uncertainty_vs_distance);  

relative_angle = 0.0:10:180
uncertainty_vs_angle =  (0.714285714285714*0.0005*exp(relative_angle/20))   
uncertainty_vs_angle(uncertainty_vs_angle > 1)=1
subplot(1,2,2); hold on; grid on; xlabel('distance (0.1m)'); ylabel('uncertainty vs angle');
plot(uncertainty_vs_angle);  

cla

%%  Camera position in the world

world_x_extent = [1 40]
world_y_extent = [1 50]
randi(360)
camera_angle = 270  ;
camera_angle = 345  ;
camera_pose = [world_x_extent(2)/2;world_y_extent(2)/2;degtorad(camera_angle)]
camera_angle_rot2 = rotz(degtorad(camera_angle))

FoV_angle = 60 ;

x_unit = [1;0]  ;
y_unit = [0;1]  ;
x_unit_origin = [camera_pose(1:2) x_unit+camera_pose(1:2) ]  ;
y_unit_origin = [camera_pose(1:2) y_unit+camera_pose(1:2) ]  ;
x_unit_rotated = [camera_pose(1:2) camera_angle_rot2(1:2,1:2)*x_unit+camera_pose(1:2) ]  ;
y_unit_rotated = [camera_pose(1:2) camera_angle_rot2(1:2,1:2)*y_unit+camera_pose(1:2) ]  ;

camera_optical_axis_direction_unit = camera_angle_rot2(1:2,1:2)*x_unit  ;
camera_optical_axis_2units = [ camera_pose(1:2) camera_pose(1:2)+camera_optical_axis_direction_unit*2]  ;


%%  Draw the camera in the world

f_camera_in_world = figure;  hold on; 
plot2_rows_rotate_robot_to_diagram(  [0 world_x_extent(2); 0 world_y_extent(2)],'bo') ; 
axis equal
plot2_rows_rotate_robot_to_diagram( x_unit_origin , 'r')    %  Always good to remember that diagram directions aren't robot directions
plot2_rows_rotate_robot_to_diagram( y_unit_origin , 'g')
plot2_rows_rotate_robot_to_diagram( x_unit_rotated , 'r')
plot2_rows_rotate_robot_to_diagram( y_unit_rotated , 'g')
plot2_rows_rotate_robot_to_diagram( camera_optical_axis_2units , 'k')

%%  FoV: angle of view is 90 degrees (or 60 degrees, or whatever)
FoV_angle = 60 ;
FoV_left_rot = rotz(degtorad(FoV_angle/2))  ;  FoV_left_rot = FoV_left_rot(1:2,1:2)  ;
FoV_right_rot = rotz(degtorad(-FoV_angle/2))  ;  FoV_right_rot = FoV_right_rot(1:2,1:2)  ;  
FoV_left_dir = FoV_left_rot*camera_optical_axis_direction_unit  ;
FoV_right_dir = FoV_right_rot*camera_optical_axis_direction_unit  ;
FoV_left_lim = [ camera_pose(1:2) FoV_left_dir+camera_pose(1:2) ]  ;
FoV_right_lim = [ camera_pose(1:2) FoV_right_dir+camera_pose(1:2) ]  ;

figure(f_camera_in_world)
plot2_rows_rotate_robot_to_diagram( FoV_left_lim , 'c')  ;
plot2_rows_rotate_robot_to_diagram( FoV_right_lim , 'm')  ;
%   cla
plot2_rows_rotate_robot_to_diagram(  [0 world_x_extent(2); 0 world_y_extent(2)],'bo')  ; 

% plot2_rows_rotate_robot_to_diagram( [ 10 ; 30 ]  , 'rx' ) 
% plot2_rows_rotate_robot_to_diagram( [ 12 15 ; 32 35 ]  , 'r' ) 

%%  redo it as homogeneous coordinates
FoV_left_rot = rotz(degtorad(FoV_angle/2))  ; 
FoV_left_dir = FoV_left_rot * e2h(camera_optical_axis_direction_unit)  ;
camera_position_h = e2h(camera_pose(1:2))  ;
FoV_left_lim = [ camera_position_h   FoV_left_dir+camera_position_h ]  ;
plot2_rows_rotate_robot_to_diagram( FoV_left_lim , 'c')  ;

%%

%  uncertainty_vs_distance = (0.714285714285714*0.0005*exp(dist))                                                   
scale_up = 10  ;  scale_down = 1/scale_up  ;
distance_uncertainty_costs = zeros(  world_x_extent(2)*scale_up,world_y_extent(2)*scale_up  )  ;

% visual check
check_dist_map = zeros(  world_x_extent(2)*scale_up,world_y_extent(2)*scale_up  )  ;
% visual check
check_angle_map = zeros(  world_x_extent(2)*scale_up,world_y_extent(2)*scale_up  )  ;
for xx_ = world_x_extent(1):1:world_x_extent(2)*scale_up
    for yy_ = world_y_extent(1):1:world_y_extent(2)*scale_up
       % angle of 2 relative to 1= atan2(v2.y,v2.x) - atan2(v1.y,v1.x)  
        diff_vec = [xx_ ;yy_] - camera_pose(1:2).*scale_up  ;
        diff_vec_magnitude_ = norm_2(diff_vec,1)  ;
        unit_diff_vec = diff_vec ./ diff_vec_magnitude_  ;   
       angle_diff = ...
           atan2(unit_diff_vec(2),unit_diff_vec(1)) ...
           - atan2(camera_optical_axis_direction_unit(2),camera_optical_axis_direction_unit(1))   ;
        angle_diff = radtodeg(angle_diff)  ;
        if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
        if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
        
        dist = norm_2(  [xx_ ;yy_] - camera_pose(1:2).*scale_up , 1  )  ;
        
        check_dist_map(xx_,yy_) = dist;
        
        distance_uncertainty_cost= (0.714285714285714*0.0005*exp(dist/10))  ;
        if distance_uncertainty_cost > 1
            distance_uncertainty_cost = 1  ;
        end
        if abs( angle_diff ) > FoV_angle/2; distance_uncertainty_cost = 1;   end;
        distance_uncertainty_costs(xx_,yy_) = distance_uncertainty_cost;
        
        
        % visual check
        check_angle_map(xx_,yy_) = abs( angle_diff ) ; 
        if abs( angle_diff ) > FoV_angle/2; check_angle_map(xx_,yy_) = 0;   end;            
    end
end


%------  Factor out the function 

scale_up_ = 10  ;
[distance_uncertainty_costs, check_dist_map, check_angle_map] ...
    = sensor_model_func(scale_up_, world_x_extent, world_y_extent, camera_optical_axis_direction_unit , camera_pose , FoV_angle)  ;


f_distance_uncertainty_costs=figure;  surf(distance_uncertainty_costs);  xlabel('x'); ylabel('y'); zlabel('uncertainty');  % axis equal; 
xlim([0 world_x_extent(2)*scale_up]); ylim([0 world_y_extent(2)*scale_up]); zlim([-0.1 1.1]);
f_distance_uncertainty_costs.Name='distance_uncertainty_costs' ;

% visual check
f_check_angle_map = figure;  surf(check_angle_map );  xlabel('x'); ylabel('y'); zlabel('check_angle_map');  % axis equal; 
f_check_angle_map.Name='f_check_angle_map ' ;
% visual check
f_check_dist_map = figure;  surf(check_dist_map);  xlabel('x'); ylabel('y'); zlabel('check_dist_map');  % axis equal; 
f_check_dist_map.Name='check_dist_map'  ;


%-----  Second camera : same world parameters, camera-specific parameters

FoV_angle_2 =80;
camera_pose_2 = camera_pose+[0 ; -5 ; degtorad(30)]  ;
rot_z = rotz(degtorad(30));  rot_z = rot_z(1:2,1:2)  ;
camera_optical_axis_direction_unit_2 = rot_z*camera_optical_axis_direction_unit  ;
[distance_uncertainty_costs_2, check_dist_map_2, check_angle_map_2] ...
    = sensor_model_func(scale_up_, world_x_extent, world_y_extent, camera_optical_axis_direction_unit_2, camera_pose_2 , FoV_angle_2)  ;


% visual check
f_distance_uncertainty_costs=figure;  surf(distance_uncertainty_costs_2);  xlabel('x'); ylabel('y'); zlabel('uncertainty');  % axis equal; 
xlim([0 world_x_extent(2)*scale_up]); ylim([0 world_y_extent(2)*scale_up]); zlim([-0.1 1.1]);
f_distance_uncertainty_costs.Name='distance_uncertainty_costs_2' ;
% visual check
f_check_angle_map = figure;  surf(check_angle_map_2 );  xlabel('x'); ylabel('y'); zlabel('check_angle_map');  % axis equal; 
f_check_angle_map.Name='f_check_angle_map_2' ;


%----- Combined cost function :  cost_c(x,y)= min(  cost_i(x,y) ) for i = 1 : num cameras 
%----- NOTE : try this with DStarMoo with one camera per cost layer
distance_uncertainty_costs_stacked = cat(3, 0.99*ones(size(distance_uncertainty_costs)), distance_uncertainty_costs, distance_uncertainty_costs_2)  ;
[ distance_uncertainty_costs_combined , camera_per_cell ] = min(distance_uncertainty_costs_stacked,[],3)  ;
f_camera_per_cell = figure; surf(squeeze(camera_per_cell) ); f_camera_per_cell.Name='camera_per_cell: 2 smart cameras'  ;

%-----  Third camera : same world parameters, camera-specific intrinsic and extrinsic parameters

FoV_angle_3 =80;
camera_pose_3 = camera_pose+[0 ; -10 ; degtorad(45)]  ;
rot_z = rotz(degtorad(30));  rot_z = rot_z(1:2,1:2)  ;
camera_optical_axis_direction_unit_3 = rot_z*camera_optical_axis_direction_unit  ;
[distance_uncertainty_costs_3, check_dist_map_3, check_angle_map_3] ...
    = sensor_model_func(scale_up_, world_x_extent, world_y_extent, camera_optical_axis_direction_unit_3, camera_pose_3 , FoV_angle_3)  ;

distance_uncertainty_costs_stacked=[] ;
distance_uncertainty_costs_stacked(1,:,:) = 0.99*ones(size(distance_uncertainty_costs)) ;

distance_uncertainty_costs_stacked(2,:,:) = distance_uncertainty_costs ;
distance_uncertainty_costs_stacked(3,:,:) = distance_uncertainty_costs_2  ;
distance_uncertainty_costs_stacked(4,:,:) = distance_uncertainty_costs_3  ;  % size(distance_uncertainty_costs_3)

%distance_uncertainty_costs_stacked = cat(3, 0.99*ones(size(distance_uncertainty_costs)), distance_uncertainty_costs, distance_uncertainty_costs_2)  ;
%[ distance_uncertainty_costs_combined , camera_per_cell ] = min(distance_uncertainty_costs_stacked,[],3)  ;
[ distance_uncertainty_costs_combined , camera_per_cell ] = min(distance_uncertainty_costs_stacked,[],1)  ;  size(distance_uncertainty_costs_combined )
f_camera_per_cell_num2 = figure; surf(squeeze(camera_per_cell) ); f_camera_per_cell_num2.Name='camera_per_cell: 3 smart cameras'  ;

%------------------------
%   Now look at it as the expected information gain per unit time, with the cameras giving independent observations/pose estimates

%   At each point in more than one FoV, can expect more information gain per unit time.
%   Take information gain = entropy reduction = uncertainty reduction 
%   Invert the uncertainty, and sum 

info_gains_negcosts_stacked = zeros(size(distance_uncertainty_costs_stacked))  ;
for cam_num_ = 1:size(info_gains_negcosts_stacked,1)
    info_gains_negcosts_stacked(cam_num_,:,:) = 1-distance_uncertainty_costs_stacked(cam_num_,:,:)  ;
end
sum_info_gains_negcosts_stacked = sum(info_gains_negcosts_stacked,1)  ;
sum_info_gains_negcosts_stacked = squeeze(sum_info_gains_negcosts_stacked)  ;
f_sum_info_gains_negcosts_stacked=figure;  surf( sum_info_gains_negcosts_stacked ); f_sum_info_gains_negcosts_stacked.Name='sum_info_gains_negcosts_stacked';

% OK, but is the dissimilarty in sensor/algorithm/angle not important ; if I stack 2 of the 3 cameras together, is their field of view really better than the
% other?
%  toy: 3 cameras, 1 point,  
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
%  order the payoffs by magnitude, use the first, then use the angular of the second to the first, then use the worst of third-to-second and third-to-first

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
f_bits_with_1_as_first=figure; idisp(squeeze(bits_with_1_as_first))  ;  f_bits_with_1_as_first.Name='cells outside a field of view, by assignment to the default uncertainty/cost';

%%  To combine two+ fields of view, handle this 'cost' as uncertainty at each point, as information gain at each point
%{
    Here treat information gain as observation from divergent angles to reduce the entropy of 
    the individual observations that is due to the error in the sensor model:  the errors correlate 
    with the direction of the target from the smart camera, so the information gain from each 
    observation/camera depends on the difference in angle to all other observations, as well as
    the distance to the target inherent in the individual sensor model.
%}
num_layers=size(index_order_of_sort,1)  ;
for xx_ = world_x_extent(1):1:world_x_extent(2)*scale_up
    for yy_ = world_y_extent(1):1:world_y_extent(2)*scale_up
        if 1 ==  index_order_of_sort(1 , xx_ , yy_) 
            payoffs_map(xx_,yy_, size(cameras,1)+1) = 0;
            payoffs_map_is_layer_1(xx_,yy_) = 1;
            continue;
        end
        camera_nums = index_order_of_sort(: , xx_ , yy_)  ;            
        camera_nums = camera_nums(camera_nums>1) ;
        payoff(1) = 1 * (1-distance_uncertainty_costs_stacked(camera_nums(1), xx_ , yy_) ) ;
        
        
        target=[xx_,yy_]  ;
        camera_1_vec = target - cameras(camera_nums(1)-1,:)  ;
        camera_1_vec = normalise_vector(camera_1_vec)  ;  %  - take this as the base ,  FOR NOW
        camera_2_vec = target - cameras(camera_nums(2)-1,:)  ;
            camera_2_vec = normalise_vector(camera_2_vec)  ;
        camera_3_vec = target - cameras(camera_nums(3)-1,:)  ; 
            camera_3_vec = normalise_vector(camera_3_vec)  ;

    angle_diff = ...
       atan2(camera_1_vec(2),camera_1_vec(1)) ...
       - atan2(camera_2_vec(2),camera_2_vec(1))   ;
    angle_diff = radtodeg(angle_diff)  ;
    if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
    if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
    angle_diff_1_2 = abs( angle_diff ) ;

    angle_diff = ...
       atan2(camera_1_vec(2),camera_1_vec(1)) ...
       - atan2(camera_3_vec(2),camera_3_vec(1))   ;
    angle_diff = radtodeg(angle_diff)  ;
    if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
    if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
    angle_diff_1_3 = abs( angle_diff ) ;

    angle_diff = ...
       atan2(camera_2_vec(2),camera_2_vec(1)) ...
       - atan2(camera_3_vec(2),camera_3_vec(1))   ;
    angle_diff = radtodeg(angle_diff)  ;
    if angle_diff > 180 ;            angle_diff = angle_diff - 360 ;     end;
    if angle_diff < -180 ;           angle_diff = angle_diff + 360 ;     end;
    angle_diff_2_3 = abs( angle_diff )  ;            
        
        payoff(2) = abs(sin( angle_diff_1_2 )) * (1-distance_uncertainty_costs_stacked(camera_nums(2), xx_ , yy_) ) ;
        payoff(3) = max(abs(sin(angle_diff_1_3)),abs(sin(angle_diff_2_3)))* (1-distance_uncertainty_costs_stacked(camera_nums(3), xx_ , yy_) ) ;
        payoffs_map(xx_,yy_,:)=[payoff sum(payoff)]  ;
        %         payoffs_map(xx_,yy_)=sum(payoff);
    end
end

f_payoffs_map = figure; surf(payoffs_map(:,:,4)); f_payoffs_map.Name='payoffs_map';
f_payoffs_map_is_layer_1 = figure; surf(payoffs_map_is_layer_1); f_payoffs_map_is_layer_1.Name='payoffs_map_is_layer_1';
    
%% AStar
% Pick two points, do A* planning, make sure that it shows the effects of the shape of the information gain.
% See e.g. /mnt/nixbig/ownCloud/project_code/plan_to_observe/plan_to_observe_2b_AStar_line_traj_mv_if_conf.m
size(payoffs_map(:,:,4))
max(max(payoffs_map(:,:,4)))
costs_map = max(max(payoffs_map(:,:,4))) - payoffs_map(:,:,4) ;
costs_map = squeeze(costs_map ) ;
f_costs_map = figure; 
s_costs_map = surf(costs_map); 
s_costs_map.EdgeColor = 'none';
%  s_costs_map.EdgeColor = [0 0 0];
f_costs_map.Name='costs_map';

start_1  = [265 ; 262]  ;
goal_1 = [144 ; 218]  ;
map_1 = costs_map ;

hold on; plot3_rows([start_1 ; 2.1], 'rx', 'LineWidth',2) 
hold on; plot3_rows([goal_1 ; 2.1], 'bd', 'LineWidth',2) 

            [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
            [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
            
size(total_path_start_to_goal__)
plot3_rows( [ total_path_start_to_goal__ ; 2.1*ones(1,size(total_path_start_to_goal__,2))] , 'm:', 'LineWidth',2)

axes_current = f_costs_map.CurrentAxes()
axes_current_cam_posn = axes_current.CameraPosition
axes_current_cam_target = axes_current.CameraTarget
axes_current.CameraUpVector
axes_current.CameraViewAngle

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
radtodeg(atan(camera_optical_axis_direction_unit(2)/camera_optical_axis_direction_unit(1)))











