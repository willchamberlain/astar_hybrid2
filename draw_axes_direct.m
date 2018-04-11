function draw_axes_direct(R, T, text_string, axis_arrow_scale, varargin)


camera_position = T;
% want to plot the camera axes ; plot is in the world coordinate system 
camera_to_world =  vertcat(horzcat( R, T ), [ 0 0 0 1]);
check_se3(camera_to_world);

    hold on;
    
    camera_axis_u = camera_to_world*[1.0*axis_arrow_scale 0 0 1]';
    camera_axis_v = camera_to_world*[0 1.0*axis_arrow_scale 0 1]';
    camera_axis_z = camera_to_world*[0 0 1.0*axis_arrow_scale 1]';
    
    if size(varargin) >= 2
        plot3( [camera_position(1,1),camera_axis_u(1)] , [camera_position(2,1),camera_axis_u(2)] , [camera_position(3,1),camera_axis_u(3)] , 'Color', 'r', varargin)
        plot3( [camera_position(1,1),camera_axis_v(1)] , [camera_position(2,1),camera_axis_v(2)] , [camera_position(3,1),camera_axis_v(3)] , 'Color', 'g', varargin)
        plot3( [camera_position(1,1),camera_axis_z(1)] , [camera_position(2,1),camera_axis_z(2)] , [camera_position(3,1),camera_axis_z(3)] , 'Color', 'b', varargin)
    else 
        plot3( [camera_position(1,1),camera_axis_u(1)] , [camera_position(2,1),camera_axis_u(2)] , [camera_position(3,1),camera_axis_u(3)] , 'Color', 'r')
        plot3( [camera_position(1,1),camera_axis_v(1)] , [camera_position(2,1),camera_axis_v(2)] , [camera_position(3,1),camera_axis_v(3)] , 'Color', 'g')
        plot3( [camera_position(1,1),camera_axis_z(1)] , [camera_position(2,1),camera_axis_z(2)] , [camera_position(3,1),camera_axis_z(3)] , 'Color', 'b')
    end
    
    
    if ~isempty(text_string) 
        text(camera_position(1,1),camera_position(2,1),camera_position(3,1), strcat(text_string,'')); %'\color{magenta} efficient pnp gauss identity');
    end
end
