function coordinate_system_as_SO3__ = coordinate_system_looking_at( direction_to_look_in , desired_up_vector)
% Calculates the _orientation_  of right-hand coordinate system with 
%  x axis aligned to direction_to_look_in, 
%  z axis aligned to desired_up_vector, and 
%  y axis normal to both : in FLU (x-foward, y-left, z-up )
%  i.e. determines an oriented frame.
    x_ = direction_to_look_in  ;
    z_ = desired_up_vector  ;     
    x_unit_ = normalise_vector(x_)  ;
    y_cross = cross(x_,z_)  ;
    y_cross_unit_ = normalise_vector(y_cross)  ;
    z_cross = cross(x_unit_, y_cross_unit_)  ;    
    z_cross_unit_ = normalise_vector(z_cross)  ;
    coordinate_system_as_SO3__ = [x_unit_, y_cross_unit_, z_cross_unit_] ; 
    coordinate_system_as_SO3__ = [ coordinate_system_as_SO3__(:,1) coordinate_system_as_SO3__(:,2)*-1 coordinate_system_as_SO3__(:,3)*-1 ]   ;
    %{
    draw_axes_direct( [x_unit_, y_cross_unit_, z_cross_unit_] ,[0 0 0]','',1)
    %}