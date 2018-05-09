%{
Generates a camera pose which has a minimum proportion of 3D points in the field of view 
- loop until pose is good 
-- generate camera pose with camera_extrinsics__place_camera_2()
-- check the proportion of 3D points in the field of view 
%}
function camera__ = camera_extrinsics__place_camera_safely_2(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max, camera_gauss_scale_, target_gauss_scale_, points_3D_,proportion_in_fov_)
    while true
        target_point = points_3D_(:,randi(size(points_3D_,2))) + (rand(([3,1]))-0.5).*target_gauss_scale_; 
        camera__ = camera_extrinsics__place_camera_2(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max, camera_gauss_scale_, target_point);
        points_2D = camera__.project(points_3D_);
        num_points_in_FoV = sum( ...
            ( points_2D(1,:)>=camera__.limits(1) & ...
              points_2D(1,:)<=camera__.limits(2) ) ...
            & ...
            ( points_2D(2,:)>=camera__.limits(3) & ...
              points_2D(2,:)<=camera__.limits(4) )  )  ;
        if num_points_in_FoV >= size(points_3D_,2)*proportion_in_fov_
            display(sprintf('camera_extrinsics__place_camera_safely_2:  break-ing: have %d points in FoV',num_points_in_FoV));            
%             
            draw_axes_direct_c(camera__.get_pose_rotation, camera__.get_pose_translation, '', 1.00, 'g' )   % draw the camera pose
            draw_axes_direct(camera__.get_pose_rotation, camera__.get_pose_translation, '', 0.75 )   % draw the camera pose
%             
            break
        else
            display (sprintf('camera_extrinsics__place_camera_safely_2:   retry: need %f percent in FoV, only have %d of %d', proportion_in_fov_ , num_points_in_FoV , size(points_3D_,2) ));
%             
            draw_axes_direct(camera__.get_pose_rotation, camera__.get_pose_translation, '', 0.75 )   % draw the camera pose
            pause
%             
        end 
    end
end    