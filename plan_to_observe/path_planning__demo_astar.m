addpath('/mnt/nixbig/ownCloud/project_code/')

%%
%{
        Get the lowest-cost over time from the target's trajectory  -->  forward-projection / envelope  of the instantaneous costs
%}
costmap_obs_dist__hist_min = min(costmap_obs_dist__hist(:,:,1:50),[],3)  ;
figure;  surf(costmap_obs_dist__hist_min)

%%

map_1 = zeros(100,100)  ;  start_1 = [2,2]  ;   goal_1 = [90,90]  ;

map_1(1:80,5) = inf  ;

map_1(2:100,7) = inf  ;

map_1(50,20:100) = inf  ;
test_name = 'test 0'

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
path_planning__draw_came_from(came_from, strcat(test_name,': ',sprintf(' factor %7.f',factor_)))    ;    
path_planning__draw_path( total_path_goal_to_start__ )    ;


%% Demo AStar

%--------------------------%

map_2 = ones(40,40)    ;
gaussian_1 = gauss2d( map_2  , 5 , [ 10,30 ] )    ;
gaussian_1 = gauss2d( map_2  , 5 , [ 15,30 ] )    ;

factor_ = 100 / max( max( gaussian_1 ) ) ;
factor_ = 1 / max( max( gaussian_1 ) ) ;
% factor_ = 1000 / max( max( gaussian_1 ) ) ;

gaussian_2 = gaussian_1 * factor_ ;

start_2 = [2,2]  ;   goal_2 = [39,39]  ;

map_2(5:6,1:35) = inf  ;
map_2(15,15:40) = inf  ;
map_2(25,1:25) = 500  ;
% gap at (25, 26)
map_2(25,27:40) = 500  ;
map_2(25,39) = 1  ;  % add gap at (25,35)
map_2(25,40) = 1  ;  % add gap at (25,40)
map_2(35,1:40) = 500  ;
map_2 = map_2 + gaussian_2 ;

test_name = 'test 1'
figure('Name','Map')
idisp( map_2 )   ;  hold on ;  plot2_rows(start_2','rx')    ;  plot2_rows(goal_2','gs') 
figure; 
surf(gaussian_2)
surf(map_2);  xlabel('x'); ylabel('y')

%%
% map_2 = costmap_total'  ;
% start_2 = round(robot_posn__hist(:,1) )' ;
% goal_2 = round(robot_posn__hist(:,21) )' ;
map_2 = costmap_total'  ;
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found

map_2 = costmap_obs_dist'  ;
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;     %--> no path found   

map_2 = costmap_obs_dist'  ;
start_2 = main_task_start_point'  ;
goal_2 = main_task_start_point' + [10 10]  ;     %--> OK: path found  - does work

% surf(costmap_obs_dist)
map_2 = costmap_obs_dist'  ;
start_2 = main_task_start_point'  ;
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> OK: path found  on simple cost map  

map_2 = costmap_total'  ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> OK: path found  on more complex cost map 


map_2 = costmap_total' .* 10 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> OK: more interesting path found  on more complex cost map 

map_2 = costmap_total' .* 20 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> OK: not much change there - cost is designed to drop off 

map_2 = costmap_total' .* 200 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> OK: not much change there - cost is designed to drop off 

map_2 = costmap_total' .* 0.99999 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> boring plan

map_2 = costmap_total' .* 1.00000 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> boring plan

map_2 = costmap_total' .* 2.00000 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> boring plan, but more exploration

map_2 = costmap_total' .* 4.00000 ;       % surf(map_2)
start_2 = main_task_start_point'  ;
goal_2 = main_task_end_point'  ;    %--> no path found
goal_2 = (main_task_end_point.*9+main_task_start_point)'  .* 0.1  ;     %--> boring plan just starts to get interesting : balance between assumed unit cost of motion and cost of not observing 

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_2, start_2, goal_2)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_2)    ;
path_planning__draw_came_from(came_from, strcat(test_name,': ',sprintf(' factor %7.f',factor_)))    ;  
path_planning__draw_path( total_path_goal_to_start__ )    ;

%%  Path shortening
%--------------------------%

map_2 = zeros(40,40)    ;
gaussian_1 = gauss2d( map_2  , 5 , [ 30,30 ] )    ;

factor_ = 100 / max( max( gaussian_1 ) ) ;

gaussian_2 = gaussian_1 * factor_ ;

start_2 = [2,2]  ;   goal_2 = [39,39]  ;

map_2(5:6,1:35) = inf  ;
map_2(15,15:40) = inf  ;
map_2(25,1:25) = 500  ;
map_2(25,27:40) = 500  ;
map_2(35,1:40) = 500  ;
B=ones( 3,3 ).*0.5  ; B(2,2)=1  ;
map_2 = conv2(map_2,B,'same')
map_2(25,1:25) = 5000  ;
map_2(25,27:40) = 5000  ;
map_2(35,1:40) = 5000  ;
map_2(35,1:20) = 10000  ;
map_2 = map_2 + gaussian_2 ;

test_name = 'test 2'
figure('Name',strcat(test_name,': ',sprintf(' factor %7.f',factor_)))
idisp( map_2 )   ;  hold on ;  plot2_rows(start_2','rx')    ;  plot2_rows(goal_2','gs') 

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_2, start_2, goal_2)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_2)    ;
path_planning__draw_came_from(came_from ,  strcat(test_name,': ',sprintf(' factor %7.f',factor_))  )    ;    
path_planning__draw_path( total_path_start_to_goal__ )    ;
total_path_smoothed__ = path_planning__smooth_path_lineofsight( total_path_start_to_goal__ , map_2 )    ;
path_planning__draw_path( total_path_smoothed__ )    ;
% path_planning__draw_path( total_path_start_to_goal__ , 'pause' )    ;
% path_planning__draw_path( total_path_goal_to_start__ , 'pause' )    ;


