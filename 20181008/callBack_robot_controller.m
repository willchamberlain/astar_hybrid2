 function callBack_robot_controller(hObject , ~)
    mousePos=get(hObject,'CurrentPoint');
    clicked_x = mousePos(1);
    clicked_y = mousePos(2);
    disp(['You clicked X:',num2str(clicked_x),',  Y:',num2str(clicked_y)]);
    figure_child_handles = guidata(gcbo);    % handle to 'application data' of current callback object 
    %  only one field in figure_child_handles at a time  - so use a struct in that field - figure_child_handles.start_point_pixels = [mousePos(1),mousePos(1)]  ;
    % figure_child_handles.start_point = ginput(2); 
    % guidata(gcbo,figure_child_handles)  % save to 'application data' of current callback object 
    figure_of_this = gcbf ;    
    figure_posn = get(figure_of_this,'Position')  ;
    width=figure_posn(3)  ;
    height=figure_posn(4)  ;
    figure__app_data_handles = guidata(gcf) 
    my_appdata_struct = figure__app_data_handles.my_appdata_struct
    pioneer2_cmd_vel_pub  = my_appdata_struct.pioneer2_cmd_vel_pub 
    pioneer2_cmd_vel_msg = my_appdata_struct.pioneer2_cmd_vel_msg
    if clicked_x > width*1/3 && (clicked_x <width*2/3 && clicked_y>height*2/3)  
        display('FORWARD')        
        send_cmd_vel_planar_safer(  0.3 , 0.0,  0.3 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
    end;
    if clicked_x > width*1/3 && (clicked_x <width*2/3 && clicked_y<height*1/3)
        display('BACKWARD')
        send_cmd_vel_planar_safer(  -0.3 , 0.0,  0.3 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
    end;
    if clicked_x < width*1/3 && (clicked_y>height*1/3 && clicked_y <width*2/3)  
        display('LEFT')
        send_cmd_vel_planar_safer(  0 , 0.2,  0.3 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
    end;
    if clicked_x > width*2/3  && (clicked_y>height*1/3 && clicked_y <width*2/3)
        display('RIGHT')
        send_cmd_vel_planar_safer(  0 , -0.2,  0.3 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
    end;
    
 end