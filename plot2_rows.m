function graphics_object_handle__ = plot2_rows(matrix_of_2D, varargin)
    graphics_object_handle__  =  plot(matrix_of_2D(1,:) , matrix_of_2D(2,:) , varargin{:});
                                                     % plot3(matrix_of_3D(1,:) , matrix_of_3D(2,:) , matrix_of_3D(3,:), varargin{:})
end
