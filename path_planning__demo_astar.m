

map_1 = zeros(100,100)  ;  start_1 = [2,2]  ;   goal_1 = [90,90]  ;

map_1(1:80,5) = inf  ;

map_1(2:100,7) = inf  ;

map_1(50,20:100) = inf  ;

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
path_planning__draw_came_from(came_from)    ;  
path_planning__draw_path( total_path_goal_to_start__ )    ;

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
map_2 = map_2 + gaussian_2 ;

idisp( map_2 )   ;  hold on ;  plot2_rows(start_2','rx')    ;  plot2_rows(goal_2','gs') 

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_2, start_2, goal_2)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_2)    ;
path_planning__draw_came_from(came_from)    ;  
path_planning__draw_path( total_path_goal_to_start__ )    ;

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

idisp( map_2 )   ;  hold on ;  plot2_rows(start_2','rx')    ;  plot2_rows(goal_2','gs') 

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_2, start_2, goal_2)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_2)    ;
path_planning__draw_came_from(came_from)    ;    
path_planning__draw_path( total_path_start_to_goal__ )    ;
total_path_smoothed__ = path_planning__smooth_path_lineofsight( total_path_start_to_goal__ , map_2 )    ;
path_planning__draw_path( total_path_smoothed__ )    ;
% path_planning__draw_path( total_path_start_to_goal__ , 'pause' )    ;
% path_planning__draw_path( total_path_goal_to_start__ , 'pause' )    ;


