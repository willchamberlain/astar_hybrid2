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


%% for DStarMoo 
addpath('/mnt/nixbig/ownCloud/project_code/3rd_party/robotics-toolbox-matlab/')
addpath('/mnt/nixbig/ownCloud/project_code/')
         %%  (3 - reduced) reduced map size

         scaledown = 5;


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
        
        floorplan_S11 = zeros(size(floorplan))  ;
        floorplan_S11(floorplan<1) = inf  ;        
        floorplan_S11(floorplan<1) = 1000000  ;        
        floorplan_S11 = floorplan_S11( 1:scaledown:end , 1:scaledown:end )  ;
        as = DstarMOO(map_1, floorplan_S11 );    % create Navigation object
        
        
        
        costs_map = max(max(payoffs_map(:,:,1))) - payoffs_map(:,:,1) ;
        costs_map = squeeze(costs_map ) ;
        %costs_map(floorplan<1) = 10*max(max(payoffs_map(:,:,1))) ;
        costs_map = costs_map(1:scaledown:end,1:scaledown:end) ;
        % payoffs_map_reduced(:,:,1) = costs_map  ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
        as.addCost(1,normA);        % add 1st add'l cost layer L
        
        costs_map = max(max(payoffs_map(:,:,2))) - payoffs_map(:,:,2) ;
        costs_map = squeeze(costs_map ) ;
        %costs_map(floorplan<1) = 10*max(max(payoffs_map(:,:,2))) ;
        costs_map = costs_map(1:scaledown:end,1:scaledown:end) ;
        % payoffs_map_reduced(:,:,2) = costs_map  ;
        normA = costs_map - min(min((costs_map)));
        normA = normA ./ max(max(normA))  ;
        as.addCost(2,normA);        % add 1st add'l cost layer L
        
        costs_map = max(max(payoffs_map(:,:,3))) - payoffs_map(:,:,3) ;
        costs_map = squeeze(costs_map ) ;
        %costs_map(floorplan<1) = 10*max(max(payoffs_map(:,:,3))) ;
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
f_payoffs_map.Name='payoffs_map(1:scaledown:end,1:scaledown:end,4)';     
 
            size(floorplan)
            size(payoffs_map) 
            %  floorplan_display_as_surf = (1-floorplan).*Vz*0.75  ;
            %  floorplan_display_as_surf = (1-floorplan).*Vz  ; 
            % s=surf(  floorplan_display_as_surf(1:scaledown:end,1:scaledown:end,1)  ); 
            % s.EdgeColor='None'
            % s.FaceColor='k'
 
%-- demo plan BETTER : use the same object as the robots : fixed goal, fixed starting point : test the time to plan
        payoffs_map_costmap_layers = payoffs_map_reduced(:,:,1:3)  ;  % CHANGE THIS TO CHANGE THE ROBOT PLANNING
        % floorplan_  %  MAKE SURE THAT THIS IS RIGHT        
        start_1  = [77  ;  32].*cells_per_metre  ;  start_1  = round(start_1 ./ scaledown) ;
        goal_1 = [81  ;  39].*cells_per_metre  ;    goal_1 = round(goal_1 ./ scaledown) ;
        demo_start_location =  start_1  ;  %  MAKE SURE THAT THIS IS RIGHT
        demo_goal_location =  goal_1  ;  %  MAKE SURE THAT THIS IS RIGHT
        % s_demo_plan = Sensor_model_as_object('s_demo_plan', floorplan_ , payoffs_map_costmap_layers , demo_start_location , demo_goal_location ,  1  ) ;        
        s_demo_plan = Sensor_model_as_object('s_demo_plan', floorplan_S11 , payoffs_map_costmap_layers , demo_start_location , demo_goal_location ,  1  ) ;                
        s_demo_plan__path =  s_demo_plan.planningStep(demo_start_location)  ;
        
        f_payoffs_map_costmap_layers = figure; f_payoffs_map_costmap_layers.Name='f_payoffs_map_costmap_layers'; 
        hold on; s=surf(payoffs_map_costmap_layers(:,:,1)); s.EdgeColor='None'  ;
        hold on; s=surf(payoffs_map_costmap_layers(:,:,2)); s.EdgeColor='None'  ;
        hold on; s=surf(payoffs_map_costmap_layers(:,:,3)); s.EdgeColor='None'  ;       
        %   hold on; s=surf(0.5*(floorplan_S11>0));  s.EdgeColor='None';  s.FaceAlpha=0.2  ;
        hold on; s=surf(0.5 + (0.5*(floorplan_S11>0))) ;  s.EdgeColor='None' ;  s.FaceAlpha=0.7  ;  s.FaceColor='k'  ;
        hold on; s=surf(0.5*(floorplan_S11>0)) ;  s.EdgeColor='None' ;  
        plot3_rows( [  [ s_demo_plan__path]' ; ones(1,size(s_demo_plan__path,1)) ]  , 'rx' )  ;
        
        f_floorplan_S11= figure; s=surf(floorplan_S11);  s.EdgeColor='None' ;  hold on; f_floorplan_S11.Name='demo plan : floorplan_S11  plus  s_demo_plan__path' ;
        plot3_rows( [  [ s_demo_plan__path]' ; max(max(floorplan_S11))*ones(1,size(s_demo_plan__path,1)) ]  , 'rx' )  ;
        
figure_named('demo plan : fixed goal, fixed starting point')                          
        figure(f_payoffs_map);  pause(1); hold on;                %plot3_rows( [ P 1.1*ones(size(P,1),1)]' , 'rx', 'LineWidth',2)  % plot flat         
         tic
         s_demo_plan.as.plan(demo_start_location,scaledown,demo_goal_location);       % setup costmap for specified goal ;  N = number of cost layers to use, where 1=distance and 2=heuristic
         toc
         
        P = s_demo_plan.as.path(start_1);    % plan solution path star-goal, return path 
        size(P)
        path_plotpoints = [ P 1.1*ones(size(P,1),1)]  ;
        figure; s=surf( payoffs_map_reduced(:,: , 4) ); s.EdgeColor='None'  ;  hold on ;
        for ii_ = 1:size(P,1)
            plot3( P(ii_,1),P(ii_,2),  payoffs_map_reduced(P(ii_,2),P(ii_,1),4)+0.01, 'rs', 'LineWidth',1)  % plot across the surface
        end
 
%-- draw the walls

        
        f_rendezvous_map = figure_named('Rendezvous','default');
        hold on
%         s_f_rendezvous_map=surf(zeros(size(payoffs_map(1:scaledown:end,1:scaledown:end,4))));
%         s_f_rendezvous_map.EdgeColor='none';
        
        size(payoffs_map_reduced)
        payoffs_map_costmap_layers = payoffs_map_reduced(:,:,1:3)  ;  % CHANGE THIS TO CHANGE THE ROBOT PLANNING
        
        s1111 = Sensor_model_as_object('s1111',floorplan_,payoffs_map_costmap_layers, start_1,goal_1,  1 , 1 , 2  ) ;        
        path = s1111.planningStep()  ;
        size(path)
        path(1,:)
        path(end,:)
        robots{1}.robot_sim = s1111  ;
        robots{1}.status = s1111.moveAStep  ;
        robots{1}.location = s1111.robot_location  ;
        robots{1}.location_hist = s1111.robot_location  ;
        
        s2222 = Sensor_model_as_object('s2222',floorplan_,payoffs_map_costmap_layers, goal_1, start_1,  1  , 1 , 1) ;       
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
        iteration = 0;
        while robot_in_play
            iteration = iteration + 1;  display(sprintf('iteration=%i',iteration));
            
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
       