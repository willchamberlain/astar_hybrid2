%{
Generates a camera pose which has a minimum proportion of 3D points in the field of view 
- loop until pose is good 
-- generate camera pose with camera_extrinsics__place_camera()
-- check the proportion of 3D points in the field of view 
%}
function camera__ = camera_extrinsics__place_camera_safely(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max, points_3D_,proportion_in_fov_)
    while true
        camera__ = camera_extrinsics__place_camera(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max);
        points_2D = camera__.project(points_3D_);
        num_points_in_FoV = sum( ...
            ( points_2D(1,:)>=camera__.limits(1) & ...
              points_2D(1,:)<=camera__.limits(2) ) ...
            & ...
            ( points_2D(2,:)>=camera__.limits(3) & ...
              points_2D(2,:)<=camera__.limits(4) )  )  ;
        if num_points_in_FoV >= size(points_3D_,2)*proportion_in_fov_
            display(sprintf('camera_extrinsics__place_camera_safely:  break-ing: have %d points in FoV',num_points_in_FoV));
            break
        else
            display (sprintf('camera_extrinsics__place_camera_safely:   retry: need %f percent in FoV, only have %d of %d', proportion_in_fov_ , num_points_in_FoV , size(points_3D_,2) ));
        end 
    end
end    