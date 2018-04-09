% Copy out the current portion of the timeseries and save it to a
% file, and reset the start point to the current point  

function separate_data_into_distinct_tracks__copy_and_save_data(bagfile_dir, bagfile_name)
    global ts_Pioneer1_amcl_pose  ; 
    global  timeseries_part_start  ;
    global  timeseries_part_end  ;
    timeseries_this_temp =  getsampleusingtime(ts_Pioneer1_amcl_pose, ts_Pioneer1_amcl_pose.Time(timeseries_part_start), ts_Pioneer1_amcl_pose.Time(timeseries_part_end));

    % filename_timepart = datetime(ts_Pioneer1_amcl_pose.Time(timeseries_part_start), 'Format', 'yyyy-MM-dd')
    filename = sprintf( ...
        '%s%s_pts_%d_to_%d.timeseries.m', ...
        bagfile_dir, bagfile_name, ...
        timeseries_part_start, timeseries_part_end  ) ;
    save( filename  ,  'timeseries_this_temp' );
    timeseries_part_start = timeseries_part_end;


% ts_Pioneer1_amcl_pose.Time(timeseries_part_start)
% 
% aDateNumber = datenum(ts_Pioneer1_amcl_pose.Time(timeseries_part_start)/(60*60))
% aDatetime = datetime(aDateNumber ,'ConvertFrom','datenum')
% datetime( aDatetime , 'Format' , 'yyyy-MM-dd'  )
% 
% ts_Pioneer1_amcl_pose.DataInfo
% ts_Pioneer1_amcl_pose.TimeInfo
% ts_Pioneer1_amcl_pose.TimeInfo.StartDate
% min(ts_Pioneer1_amcl_pose.Time)
% ts_Pioneer1_amcl_pose.TimeInfo.getTimeStr
% 
% ts_Pioneer1_amcl_pose.TimeInfo.Format = 'yyyy_mm_dd_HHMMSS'
% ts_Pioneer1_amcl_pose.TimeInfo.Format
% 
% ts_Pioneer1_amcl_pose.TimeInfo.Units

%{
https://au.mathworks.com/help/matlab/ref/timeseries-class.html#bsplqwl

A time vector of a timeseries object can be either numerical (double) values or valid 
MATLAB date strings.

When the timeseries 
    TimeInfo.StartDate property is empty, the 
        numerical time values are measured 
        relative to 
        0 (or another numerical value) 
        in specified units. In this case, 
    the time vector is described as "relative" (that is, 
        it contains time values that are 
        not associated with a specific start date).

When 
    TimeInfo.StartDate is nonempty, the 
        time values are 
        date strings measured 
        relative to 
            StartDate 
        in specified units. In this case, 
    the time vector is described as "absolute" (that is, 
        it contains time values that 
        are associated with a 
        specific calendar date). 

%}

end


