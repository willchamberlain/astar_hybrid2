function graphics_object_handle__ = plot2_rows_rotate_robot_to_diagram(matrix_of_2D, varargin)
    rotate_by = rotz(degtorad(90))  ;
    if 2 == max(max(size(matrix_of_2D)))
        rotate_by = rotate_by(1:2,1:2)  ;        
    end
    matrix_of_2D_rotated =  rotate_by*matrix_of_2D  ;
%     matrix_of_2D_rotated = [ matrix_of_2D_rotated(1,:).*-1 ; matrix_of_2D_rotated(2,:) ]  ;
    graphics_object_handle__  =  plot(matrix_of_2D_rotated(1,:) , matrix_of_2D_rotated(2,:) , varargin{:});
                                                     % plot3(matrix_of_3D(1,:) , matrix_of_3D(2,:) , matrix_of_3D(3,:), varargin{:})
end
