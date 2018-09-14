 function callBack_geometric_representation_graph_of_FoV(hObject , ~)
    mousePos=get(hObject,'CurrentPoint');
    disp(['You clicked X:',num2str(mousePos(1)),',  Y:',num2str(mousePos(2))]);
    figure_child_handles = guidata(gcbo);
    figure_child_handles.start_point_pixels = [mousePos(1),mousePos(1)]  ;
    figure_child_handles.start_point = ginput(2); 
    %     figure_child_handles
    guidata(gcbo,figure_child_handles) 
 end