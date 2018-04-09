function camera_extrinsics_data_analysis__keyboard_callback( src , event )
    global fig_handle
    global pixels_2D
    global points_3D
%     global start_idx
    global current_idx
    global datapoint_handles_2D ; 
    global datapoint_handles_3D ; 

    
    disp(event.Key);
    if strcmp(event.Key,'leftarrow')
       disp('LEFT ARROW')
    elseif strcmp(event.Key,'rightarrow')
       disp('RIGHT ARROW')
    elseif strcmp(event.Key,'downarrow')        
        disp(  sprintf( 'current_idx=%d' , current_idx )  )         
        figure(fig_handle)                
            subplot(2,1,2); hold on;    
                datapoint_handles_2D(current_idx) = plot( ...
                   pixels_2D( current_idx , 1 )  ,  ...
                   max(pixels_2D(:,2))-pixels_2D(current_idx,2)'  ,  ...           
                    'bx');        
            subplot(2,1,1); hold on;
                datapoint_handles_3D(current_idx) = plot3( ...
                   points_3D( current_idx , 1 )  ,  ...
                   points_3D( current_idx , 2 )  ,  ...
                   points_3D( current_idx , 3 )  ,  ...
                    'rd');
        current_idx = current_idx + 1  ;
    elseif strcmp(event.Key,'uparrow')
        disp(  sprintf( 'current_idx=%d' , current_idx )  )       
        figure(fig_handle)                
            subplot(2,1,2); hold on;    
                delete(datapoint_handles_2D(current_idx))   %  remove the graphics object : saves memory 
            subplot(2,1,1); hold on;
                delete(datapoint_handles_3D(current_idx))   %  remove the graphics object : saves memory 
        current_idx = current_idx - 1  ;
   end
end



