function graphics_object_handle__ = plot2_rows(matrix_of_2D, varargin)
    graphics_object_handle__  =  plot(matrix_of_2D(1,:) , matrix_of_2D(2,:) , varargin{:});
end
