

map_1 = zeros(100,100)  ;  start_1 = [2,2]  ;   goal_1 = [90,90]  ;

map_1(1:80,5) = inf  ;

map_1(2:100,7) = inf  ;

map_1(50,20:100) = inf  ;

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_1, goal_1)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_1)    ;
path_planning__draw_came_from(came_from)    ;  
path_planning__draw_path( total_path_goal_to_start__ )    ;
