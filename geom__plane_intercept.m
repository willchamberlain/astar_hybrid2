function [z_plane_intercept_vec__ , scaling_factor_for_vector_along_line_to_intercept__] ...
    = geom__plane_intercept(  l_point_on_line_  ,  line_direction_vector_  , normal_to_plane_)
%% camera optical axis intercept on the floor plain 
% simplified case of the general line-plane intercept 
% camera_to_world = camera.T(1:3,1:3)  ; 
% camera_axis_z = camera_to_world*[0  0 1.0 ]'  ;
% 
% % Line 
% l_point_on_line_ =   camera.T(1:3,4)  ; %  camera position is a point on the camera optical axis
l_vector_along_line = line_direction_vector_  ; % vector in direction of the line 
%d = % scalar, which extends the line from  l_o  parallel to  l  for some distance (in this until it intercepts the plane).

% Plane :  xy plane in this case
point_on_plane_xy = l_point_on_line_(1:2);
point_on_plane =  [ point_on_plane_xy ; 0 ]  ; %  a point on the plane: use camera position with z=0 ; it is as good as any other
% p = % set of points in the plane

scaling_factor_for_vector_along_line_to_intercept__ = ...
    dot( (point_on_plane - l_point_on_line_ ) , normal_to_plane_ ) ...
    / ...
    dot( l_vector_along_line , normal_to_plane_ )  ;

z_plane_intercept_vec__ = scaling_factor_for_vector_along_line_to_intercept__*l_vector_along_line + l_point_on_line_  ;


% % plot the (z=0)-intercept point 
% plot3_rows( scaling_factor_for_vector_along_line_to_intercept__*l_vector_along_line + l_point_on_line_ , 'rs' )  ;
% text( z_plane_intercept(1), z_plane_intercept(2) , 'z_intercept on [x y 0]' )
% % plot a patch of the z=0 plane 
% handle_patch = patch( [ -10 -10 10 10 ]' , [ -10 10 10 -10 ]' , [ 0 0 0 0 ] )  ;
% alpha(handle_patch, 0.3)  ;
end