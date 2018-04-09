
%{ 
purpose:  step through the data from a test-run file, and separate the data out into distinct 
runs (left-to-right, right-to-left, etc)

Idea 1: 
    plot the full path from the file. 
    label the data points sequentially under user input
        take keyboard input
            step-through
                right cursor steps forward one measurement
                left cursor steps back one measurement
                show the number associated with all data points from start of the current segment to the current time point
            separate
                array of segment start-end numbers - numbers of datapoints in the original
                set by keyboard input to console - easier and more flexible than dicking with a GUI 
                on change, redraw full set of tracks 
                each track has a different colour
                only current track is numbered           
%}

%%
% EXAMPLE - draw an ANIMATED plot with the comet() function 
t = 0:.01:2*pi;
x = cos(2*t).*(cos(t).^2);
y = sin(2*t).*(sin(t).^2);
comet(x,y);

%%
% EXAMPLE - refreshdata
figure('Name','Refreshdemo')
z = peaks(5);
[c h] = contour(z,'ZDataSource','z');
drawnow
pause(3) % Wait 3 seconds and the graph will update
z = peaks(20);
refreshdata(h)

%%
% EXAMPLE - Keyboard input to step through the points.
%  https://au.mathworks.com/matlabcentral/answers/335596-how-to-make-matlab-detect-keyboard-stroke
%  https://au.mathworks.com/matlabcentral/answers/136897-use-keyboard-input-to-assign-value-to-a-variable
addpath('/mnt/nixbig/ownCloud/project_code/')
h_fig = figure;
set(h_fig,'KeyPressFcn',@separate_data_into_distinct_tracks_keyboard_callback);

%%
rosshutdown
setenv( 'ROS_HOSTNAME' , '131.181.33.193' )
rosinit('131.181.33.193')
rosinit('131.181.33.193', 'NodeHost','131.181.33.193','NodeName','/test_node')

%%
% load the timeseries and plot the whole path
%  see /mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/rosbag_parse_as_timeseries_c.m
addpath('/mnt/nixbig/ownCloud/project_code/')

bagfile_dir = '/mnt/nixbig/data/project_AA1__2_extrinsics_calibration/2018_02_08_bags/';
bagfile_name = '2018-02-14-11-32-51_0.bag';
bag = rosbag(sprintf('%s%s',bagfile_dir,bagfile_name));
h_fig_timeseries = figure( 'Name' , sprintf( 'VOS estimated pose vs Pioneer lidar pose - XY plane - %s' , bagfile_name ) );  
grid on; hold on; axis equal; xlabel('x'); ylabel('y');
addpath('/mnt/nixbig/ownCloud/project_code/')
set(h_fig_timeseries,'KeyPressFcn',@separate_data_into_distinct_tracks_keyboard_callback);

bagselect_Pioneer1_amcl_pose = select(bag, 'Topic', '/amcl_pose'); 

global ts_Pioneer1_amcl_pose;
ts_Pioneer1_amcl_pose = timeseries(bagselect_Pioneer1_amcl_pose, 'Pose.Pose.Position.X', 'Pose.Pose.Position.Y');

plot(ts_Pioneer1_amcl_pose.Data(:,1), ts_Pioneer1_amcl_pose.Data(:,2), 'r');

%%
% use the keyboard to show the datapoints one by one
%  https://au.mathworks.com/help/matlab/ref/timeseries.getsampleusingtime.html
ts_Pioneer1_amcl_pose.TimeInfo.Start;
class(ts_Pioneer1_amcl_pose.TimeInfo.End);
ts_Pioneer1_amcl_pose.TimeInfo.Start + 1000000.00;
ts_Pioneer1_amcl_pose.TimeInfo.End - ts_Pioneer1_amcl_pose.TimeInfo.Start;
ts_Pioneer1_amcl_pose_part001 = getsampleusingtime(ts_Pioneer1_amcl_pose,ts_Pioneer1_amcl_pose.Time(1),ts_Pioneer1_amcl_pose.Time(1)) ;
class(ts_Pioneer1_amcl_pose.Time);
ts_Pioneer1_amcl_pose.Time;
%- 
global  timeseries_part_start  ;
timeseries_part_start = 1  ;
global  timeseries_part_end  ;
timeseries_part_end = 0  ;      %  start at zero because the callback func separate_data_into_distinct_tracks_keyboard_callback has to increment before use
%- 
global ts_Pioneer1_amcl_pose_point001 ;
ts_Pioneer1_amcl_pose_point001 = getsampleusingtime( ...
    ts_Pioneer1_amcl_pose , ...
    ts_Pioneer1_amcl_pose.Time(1) , ...
    ts_Pioneer1_amcl_pose.Time(1) )     ;

%%
%  execute manually on the console
separate_data_into_distinct_tracks_copy_and_save_data(bagfile_dir, bagfile_name)


%%
% Copy (not cut) the timeseries data 
%   -  from timeseries_part_end_start to timeseries_part_end_end
%   -  to a new timeseries
%   -  and save it to file 


%%
%  timetable  -  Time-stamped data in tabular form  -  https://au.mathworks.com/help/matlab/timetables.html 
%  timetables provide time-specific functions to align, combine, and perform calculations with one or more timetables
%   e.g. sortrows , synchronize , 

%% 
% Copy out the current portion of the timeseries and save it to a file, and reset the start point to the current point.  
%       /mnt/nixbig/ownCloud/project_code/separate_data_into_distinct_tracks_copy_and_save_data.m
%
%       !!  fts2ascii  ,  ascii2fts  !!  -  are only for _financlial time   data
%

separate_data_into_distinct_tracks_copy_and_save_data()

%%
% EXAMPLE - cell array      -       https://au.mathworks.com/help/matlab/matlab_prog/cell-vs-struct-arrays.html
%   Q: Can I save a cell array to a sensible text format? 
%   A: Nope : comes out as binary 
%   +ve:    can use cell array as an index into the timeseries:  timeseries date -  datapoint start -  datapoint end - timeseries filename
%   -ve:    have to use Matlab to read the file / act as database
temperature(1,:) = {'2009-12-31', [45, 49, 0]};
temperature(2,:) = {'2010-04-03', [54, 68, 21]};
temperature(3,:) = {'2010-06-20', [72, 85, 53]};
temperature(4,:) = {'2010-09-15', [63, 81, 56]};
temperature(5,:) = {'2010-12-09', [38, 54, 18]};

%%
% EXAMPLE - table           -       
%   array2table , cell2table , struct2table , table2array , table2cell , table2struct 
%  readtable  ->  table2cell  -> work with cell array -> cell2table -> writetable
temperature_table = cell2table(temperature);
writetable( temperature_table , '/mnt/nixbig/ownCloud/project_code/separate_data_into_distinct_tracks_keyboard_callback__example_table.txt' )
temperature_table_reread = readtable('/mnt/nixbig/ownCloud/project_code/separate_data_into_distinct_tracks_keyboard_callback__example_table.txt')
temperature_table_reread.Properties
temperature_table_reread.Properties.RowNames
temperature_table_reread.Properties.VariableNames
temperature_table_reread.Properties.DimensionNames
class(temperature_table_reread.temperature1)                % cell  -->  string 
temperature_table_reread.temperature1                           % column access 
class(temperature_table_reread.temperature2_1)              % double
temperature_table_reread{1,2:4}                                         % row-ish access: need to specify row _AND_ column indexes, _AND_ need to access fields of the same type (e.g. double)

temperature_table_reread_cells = table2cell(temperature_table_reread)
temperature_table_reread_cells{1}                                       % linear index
temperature_table_reread_cells{:,1}                                     % column access
temperature_table_reread_cells{1,:}                                     % row access - heterogeneous types allowed 
row_1 = temperature_table_reread_cells{1,:}                                     % only returns _one_ cell : the first
class(row_1)
row_1
row_1 = temperature_table_reread_cells{1,2:end}                                     % only returns _one_ cell : the first
row_1 = temperature_table_reread_cells{1,2:3}                                     % only returns _one_ cell : the first
cell2array

temperature_table_reread_struct = table2struct(temperature_table_reread)
temperature_table_reread_struct(1)                                  % first struct
temperature_table_reread_struct(1,2)                               % NO - CAN'T
temperature_table_reread_struct(1).temperature2_1 + temperature_table_reread_struct(1).temperature2_2   %  https://au.mathworks.com/help/matlab/matlab_prog/access-data-in-a-structure-array.html









