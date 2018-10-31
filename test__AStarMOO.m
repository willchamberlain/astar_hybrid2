% Example 2::
       goal = [90;90];
       start = [80;80];
       as = Astar(zeros(100));          % create Navigation object with pseudo-
                               % random occupancy grid
        terrain = rand([100,100]);        
        terrain( terrain<0.9  )=0;
        terrain = ismooth(terrain,5) ;
        terrain = terrain-min(terrain(:));
        terrain = terrain* (1/max(terrain(:))) ;
        histogram(terrain)        
        surf(terrain)
        
        terrain = rand([100,100]);        
        terrain( terrain<0.9  )=0;
        terrain = terrain-min(terrain(:));
        terrain = terrain* (1/max(terrain(:))) ;
        
       as.addCost(terrain);    % terrain is a 100x100 matrix of 
%                               % elevations [0,1]

        % N: number of optimization objectives; standard A* is 2   (i.e. distance and heuristic)
        %   layers: number of cost layers in costmap
        %   algorithm: specify standard A*(0), A*-MOO (1), A*-PO (2)
        as.reset()
 	    as.plan(goal,2,2,0);    % setup costmap for specified goal
 	    as.plan(goal,3,4,1);    % setup costmap for specified goal
        
%                               % (3 and 4 include the added terrain cost)
       as.path(start);         % plan solution path start-goal, animate
       P = as.path(start);     % plan solution path start-goal, return 
       size(P)
       
%                               % path


       goal = [90;90];
       start = [10;30];
       as = Astar(zeros(100));   
       as.addCost(terrain);    
       as.reset() 	    
%  	   as.plan(goal,3,4,1);    % costmaps: 1=occupancy grid 2=heuristic 3=distance 4+= additional costs
%        P = as.path(goal);     % plan solution path start-goal, return 
%        as.path(goal)
 	   as.plan(goal,3,4,1);    % costmaps: 1=occupancy grid 2=heuristic 3=distance 4+= additional costs
       as.path(start)
       as.path(goal-[1; 1])
       
       % what's going on?  With Astar.m the start and goal have no entry in the backpath b map --> exits immediately 
       %   So :  check which in the neighbourhood is the next to visit.
       b=as.get_b;
       figure; surf(as.get_b); hold on;
        plot3(start(1),start(2) , b(start(2),start(1)) , 'gs')
        plot3([ start(1) start(1) ], [start(2) start(2) ] , [0 max(max(b)) ], 'g')
        
        plot3(goal(1),goal(2) , b(goal(2),goal(1)) , 'rs')
        plot3([ goal(1) goal(1) ], [goal(2) goal(2) ] , [0 max(max(b)) ], 'r')
        
        sub2ind(size(as.get_occgrid), goal(2),goal(1))  --> 8990 
        b(goal(2),goal(1))  -->  8889
        b(8889) --> 0 
        b(8888) --> 8889
        b(goal(2)-2:goal(2)+2,goal(1)-2:goal(1)+2)
       
        [i,j]=find(b(start(2)-2:start(2)+2,start(1)-2:start(1)+2) == max(max(b(start(2)-2:start(2)+2,start(1)-2:start(1)+2))))
        start(2)-2+i
        start(1)-2+j
        plot3(start(1)-2+j,start(2)-2+i , b(start(2)-2+j,start(1)-2+i) , 'gd')
       
        [i,j]=find( b( goal(2)-2:goal(2)+2,goal(1)-2:goal(1)+2) == min(min(b(goal(2)-2:goal(2)+2,goal(1)-2:goal(1)+2)))  )
        goal(2)-2+i
        goal(1)-2+j