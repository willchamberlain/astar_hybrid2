function separate_data_into_distinct_tracks__keyboard_callback( src , event )
    global timeseries_part_end;
    global ts_Pioneer1_amcl_pose;
    global ts_Pioneer1_amcl_pose_point001;
    global datapoint_handles ; 

    disp(event.Key);
    if strcmp(event.Key,'leftarrow')
       disp('LEFT ARROW')
    elseif strcmp(event.Key,'rightarrow')
       disp('RIGHT ARROW')
    elseif strcmp(event.Key,'downarrow')
        timeseries_part_end = timeseries_part_end + 1  ;
        disp(  sprintf( 'timeseries_part_end=%d' , timeseries_part_end )  ) 
        ts_Pioneer1_amcl_pose_point001 = getsampleusingtime( ...  % get a single point 
            ts_Pioneer1_amcl_pose , ...
            ts_Pioneer1_amcl_pose.Time(timeseries_part_end) , ...
            ts_Pioneer1_amcl_pose.Time(timeseries_part_end) );        
        datapoint_handles(timeseries_part_end) = plot( ...
            ts_Pioneer1_amcl_pose_point001.Data( 1 , 1 )  ,  ...
            ts_Pioneer1_amcl_pose_point001.Data( 1 , 2 )  ,  ...
            'rd');
    elseif strcmp(event.Key,'uparrow')
        timeseries_part_end = timeseries_part_end - 1  ;
        disp(  sprintf( 'timeseries_part_end=%d' , timeseries_part_end )  ) 
        ts_Pioneer1_amcl_pose_point001 = getsampleusingtime( ...  % get a single point 
            ts_Pioneer1_amcl_pose , ...
            ts_Pioneer1_amcl_pose.Time(timeseries_part_end+1) , ...
            ts_Pioneer1_amcl_pose.Time(timeseries_part_end+1) );        
%         plot(   ts_Pioneer1_amcl_pose_point001.Data( 1 , 1 )  ,   ts_Pioneer1_amcl_pose_point001.Data( 1 , 2 )  ,  'kd');
        delete(datapoint_handles(timeseries_part_end+1))   %  remove the graphics object : saves memory 
   end
end



