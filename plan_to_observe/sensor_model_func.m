function [distance_uncertainty_costs, check_dist_map, check_angle_map] = sensor_model_func(scale_up_, world_x_extent, world_y_extent, camera_optical_axis_direction_unit , camera_pose , FoV_angle )
%  scale_up_ number of cost function cells per world meter e.g. 10
%  world_x_extent, world_y_extent 2-element vectors e.g. [1,40] [1,50]
%  camera_optical_axis_direction_unit unit vector in direction of the camera optical axis 
%  camera_pose vector of camera pose with x as first element and y as second: other elements are not used
%
%  uncertainty_vs_distance = (0.714285714285714*0.0005*exp(dist))                                    
%
%  Refactored from /mnt/nixbig/ownCloud/project_code/plan_to_observe/sensor_model.m
scale_up = scale_up_  ;  scale_down = 1/scale_up  ;
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