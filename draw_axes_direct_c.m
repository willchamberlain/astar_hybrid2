function draw_axes_direct_c(R, T, text_string, axis_arrow_scale, colour)


camera_position = T;
% want to plot the camera axes ; plot is in the world coordinate system 
camera_to_world =  vertcat(horzcat( R, T ), [ 0 0 0 1]);
check_se3(camera_to_world);

    hold on;
    
    camera_axis_u = camera_to_world*[1.0*axis_arrow_scale 0 0 1]';
    plot3( [camera_position(1,1),camera_axis_u(1)] , [camera_position(2,1),camera_axis_u(2)] , [camera_position(3,1),camera_axis_u(3)] , colour)
    
    camera_axis_v = camera_to_world*[0 1.0*axis_arrow_scale 0 1]';
    plot3( [camera_position(1,1),camera_axis_v(1)] , [camera_position(2,1),camera_axis_v(2)] , [camera_position(3,1),camera_axis_v(3)] , colour)
    
    camera_axis_z = camera_to_world*[0 0 1.0*axis_arrow_scale 1]';
    plot3( [camera_position(1,1),camera_axis_z(1)] , [camera_position(2,1),camera_axis_z(2)] , [camera_position(3,1),camera_axis_z(3)] , colour )
    
    
    if ~isempty(text_string) 
        if strcmpi(class(text_string), 'char')
            text(camera_position(1,1),camera_position(2,1),camera_position(3,1), strcat(text_string,'')); %'\color{magenta} efficient pnp gauss identity');
        elseif strcmpi(class(text_string), 'cell')
            disp(sprintf('draw_axes_direct_c: size(text_string,2)=%d',size(text_string,2)));
            if size(text_string,2) > 0
                text(camera_position(1,1),camera_position(2,1),camera_position(3,1), strcat(text_string(1),'')); %'\color{magenta} efficient pnp gauss identity');
            end
            if size(text_string,2) > 1
                text(camera_axis_u(1,1),camera_axis_u(2,1),camera_axis_u(3,1), strcat(text_string(2),'')); %'\color{magenta} efficient pnp gauss identity');
            end
            if size(text_string,2) > 2
                text(camera_axis_v(1,1),camera_axis_v(2,1),camera_axis_v(3,1), strcat(text_string(3),'')); %'\color{magenta} efficient pnp gauss identity');
            end
            if size(text_string,2) > 3
                text(camera_axis_z(1,1),camera_axis_z(2,1),camera_axis_z(3,1), strcat(text_string(4),'')); %'\color{magenta} efficient pnp gauss identity');
            end
        else
            disp(sprintf('draw_axes_direct_c: class of text_string not recognised = %s',class(text_string)))
        end        
    end
    
end
