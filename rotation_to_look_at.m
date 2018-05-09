%{
http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions/
Something wrong here - look at  https://www.khronos.org/opengl/wiki/GluLookAt_code  instead 
%}

function quaterion__ = rotation_to_look_at( current_camera_forward_vector  ,   direction_to_look_in ,  desired_up_vector ) 
    %{
    x_ = [ 3 -4 -1 ]'
    y_ = rotz(degtorad(90)) * [ x_(1) x_(2) 0]'
    z_cross = cross(x_, y_)
    %}
    
    %  determine an oriented frame 
    x_ = direction_to_look_in  ;
    z_ = desired_up_vector  ;     
    x_unit_ = normalise_vector(x_)  ;
    y_cross = cross(x_,z_)  ;
    y_cross_unit_ = normalise_vector(y_cross_)  ;
    z_cross = cross(x_unit_, y_cross_unit_)  ;    
    z_cross_unit_ = normalise_vector(z_cross)  ;
    %{
    draw_axes_direct( [x_unit_, y_cross_unit_, z_cross_unit_] ,[0 0 0]','',1)
    %}
    
    
    
    
    forward_axis_vector_ = [ 3 -4 -1 ]'
    left_axis_vector_ = rotz(degtorad(90)) * [ forward_axis_vector_(1) forward_axis_vector_(2) 0]'
    up_axis_vector_ = cross(x_, left_axis_vector_)
    
    
    current_camera_forward_vector = normalise_vector(current_camera_forward_vector);
    direction_to_look_in = normalise_vector(direction_to_look_in);
    desired_up_vector = normalise_vector(desired_up_vector);
    quat_rot1 = rotation_between_vectors(  current_camera_forward_vector , direction_to_look_in )  ;    
    %  Recompute desiredUp so that it's perpendicular to the direction
    % Can skip that part if you really want to force desiredUp
    right_vector = cross( direction_to_look_in , desired_up_vector )  ;
    desired_up_vector = cross( right_vector , direction_to_look_in )  ;
    %  Because of the 1rst rotation, the up is probably completely screwed up.
    %  Find the rotation between the "up" of the rotated object, and the desired up
    newUp = quat_rot1*desired_up_vector  ;
    quat_rot2 = rotation_between_vectors(  newUp , desired_up_vector)  ;
    quat_targetOrientation = quat_rot2 * quat_rot1 ; %  remember, in reverse order.

    quat_targetOrientation =  quat_rot1 ; 
    quaterion__ = quat_targetOrientation  ;
end