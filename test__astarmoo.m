%  addpath( '/mnt/nixbig/downloads/RobotciVisionToolbox-GIT/master/robotics-toolbox-matlab/astar' )
%%
addpath('/mnt/nixbig/ownCloud/project_code/3rd_party/robotics-toolbox-matlab/')

%%
world_occmap = zeros(20,30)  ;
start = [ 12 ; 12 ]  ;
goal = [ 20 ; 20 ]  ;

moo = AstarMOO(world_occmap)  ;

moo = AstarMOO( 1.0*(magic(50)>900) )

num_optimisation_objectives = 2 ;   %  N: number of optimization objectives; standard A* is 2
moo.plan( goal , num_optimisation_objectives )  ;
moo.path( start )  ;               % plan solution path start to goal, animate
path_ = moo.path( start )  ;    % plan solution path start to goal


         goal = [100;100];
         start = [1;1];
         as = AstarMOO(0);   % create Navigation object with random occupancy grid
         as.addCost(1,L);    % add 1st add'l cost layer L
         as.plan(goal,3);    % setup costmap for specified goal
         as.path(start);     % plan solution path start-goal, animate
         P = as.path(start);    % plan solution path start-goal, return path

%%

a = normc( conv2(abs(randn(50)),ones(5))  )  ;
a = normc( a' )  ;
size(a)
surf(a)

aspo = AstarPO(a) 
goal = [50;30]  ;
start = [20;10]  ;
as.plan(goal,2)  ;   % setup costmap for specified goal
as.path(start)  ;    % plan solution path star-goal, animate
P = as.path(start)  ; % plan solution path star-goal, return path
hold on; plot2_rows( P' )

as.path(  [ 30 ; 80 ]  )  
P = as.path(  [ 30 ; 80 ]  )
hold on; plot2_rows( P' )  

as.path(  [ 60 ; 90 ]  )  
P = as.path(  [ 60 ; 90 ]  )
hold on; plot2_rows( P' )  

as.path(  [ 10 ; 90 ]  )  
P = as.path(  [ 10 ; 90 ]  )
hold on; plot2_rows( P' )  



