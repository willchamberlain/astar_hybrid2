

%% find and draw the intercept with the plane at the height of the robot.

cam_1_3d=[cam_1 2] ;
cam1_fov_3d=[cam1_fov zeros(size(cam1_fov,1),1) ] ;

cam_2_3d=[cam_2 2] ;
cam2_fov_3d=[cam2_fov zeros(size(cam2_fov,1),1) ] ;

cam_3_3d=[cam_3 2] ;
cam3_fov_3d=[cam3_fov zeros(size(cam3_fov,1),1) ] ;


figure_2d_handle = figure_named('cam_ cam1_fov 2D'); grid on; hold on; axis equal;
figure_3d_handle = figure_named('cam_ cam1_fov 3D'); grid on; hold on; axis equal;
for ii_ = 1:size(cam1_fov_3d,1)
    cam_to_floor_vec_ends(:,:,ii_) = [  cam_1_3d ;  cam1_fov_3d(ii_,:)]'  ;
    cam_to_floor_vec(ii_,:) = -cam1_fov_3d(ii_,:) + cam_1_3d  ;
    cam_to_floor_vec(ii_,:) = cam_1_3d  - cam1_fov_3d(ii_,:)  ;
    plot3_rows( cam_to_floor_vec_ends(:,:,ii_) )  ;
    if ii_ > 1
        plot3_rows( [ cam1_fov_3d(ii_,:) ;cam1_fov_3d(ii_-1,:) ]' )  ;
    else
        plot3_rows( [ cam1_fov_3d(ii_,:) ; cam1_fov_3d(size(cam1_fov_3d,1),:) ]' )  ;        
    end
    
    %
    [z_plane_intercept_vec__ , ...
        scaling_factor_for_vector_along_line_to_intercept__] ...
        =   geom__z_plane_intercept(  [cam_1_3d(1:2)' ; 0]  ,  cam_to_floor_vec(ii_,:)'  , 1)   ;
    plot3_rows( [cam_1_3d']-z_plane_intercept_vec__, 'bs')
end


% Draw FoV - if I'd derived the FoV from the ray projection, this would be the envelope on 
% the set of rays - but reintroduce uncertainty and that becaomes a lot of rays..


% this is like finding the configuration space with two superimposed sets of obstacles - one at floor level, 
%  the other at the height of the top of the robot


% what's the cut of the fustrum at height r_h above the floor? 
% it's the field of view as it intercepts the plane parallel to the floor at r_h 
% try /mnt/nixbig/ownCloud/project_code/geom__plane_intercept.m  and  /mnt/nixbig/ownCloud/project_code/geom__z_plane_intercept.m
%       [z_plane_intercept_vec__ , scaling_factor_for_vector_along_line_to_intercept__] ...
%             = geom__z_plane_intercept(  l_point_on_plane_  ,  line_direction_vector_  , height_)

%{
Comes back to the same question as with the camera calibration - 
what volume are the robot features visible to teh camera
PLUS I should ask the question - what about the extent of the feature ~ volumes moving through volumes.
%}
%{
If need to see features at the top and the bottom, is the intersection of the areas projected onto the floor
    or equivalently the robot feature volumes remain within the observation volume.
If need to see any of the features, is the union of the areas projected onto the floor
    or equivalently some subset of the robot feature volumes remain within the observation volume.
Question for Anders.
and/or Keeping points within plane constraints.
%}


