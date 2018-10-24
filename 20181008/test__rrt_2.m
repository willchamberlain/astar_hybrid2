function test__rrt_2()

addpath('/mnt/nixbig/ownCloud/project_code/')
addpath('/mnt/nixbig/ownCloud/project_code/20181008')
addpath('/mnt/nixbig/ownCloud/project_code/plan_to_observe/')

%%
%{

See exp_cups_S11_simplified.m

%}


%%
      x = -4:4; y = [0 .15 1.12 2.36 2.36 1.46 .49 .06 0];
      cs = spline(x,[0 y 0]);
      xx = linspace(-4,4,101);
      plot(x,y,'o',xx,ppval(cs,xx),'-');
      
      x = 0:4; y = [0 .00 1.0 3.0 4.0];
      cs = spline(x,[0 y 0]);
      xx = linspace(0,4,51);
      plot(x,y,'o',xx,ppval(cs,xx),'-');
%%

%  1) map
%  2) robot(s) that want to move
%  3) robot(s) that can help but have main goals
%  4) obstacles --> occlusions  -  DO
%  5) plan AStar --> approximate cost/benefit
%  6) smooth path --> integrate cost/benefit by simulation  -  DO
%  7) plan navmesh of visible spaces 
%  8.1)  location of target robot(s) and immediate plans --> uncertainty regions
%  8.2)  BWDIST from uncertainty region + occlusion --> BWDist in navmesh --> isosurfaces --> multiple cost layers --> pareto --> robustness


      
      
      map_1 = zeros(40,50);
      map_x_extent=size(map_1,1);
      map_y_extent=size(map_1,2);
      walls{1} = Wall( inf(2,6), [7;7]) ;
      walls{2} = Wall( inf(2,6), [27;7]) ;
      walls{3} = Wall( inf(3,3), [35;33]) ;
      walls{4} = Wall( inf(3,3), [22;33]) ;
      walls{5} = Wall( inf(3,3), [9;33]) ;
      for ii_ = 1:size(walls,2)
          wall = walls{ii_};
          map_1( ...
            wall.location(1):wall.location(1)+size(wall.shape,1)-1 , ...
            wall.location(2):wall.location(2)+size(wall.shape,2)-1 ...
            ) = wall.shape;
      end
    map_1_base_zero = double(map_1 > 0.00001)  ;
    dist_func = bwdist(map_1_base_zero) ;
    
    idisp(map_1)
    figure; hold on; grid on; xlabel('x');  ylabel('y')
    surf(dist_func)

clear('worker_robots')    
worker_robots{1} = SimpleRobot()  ;
worker_robots{2} = SimpleRobot()  ;
worker_robots{3} = SimpleRobot()  ;
% worker_robots{4} = SimpleRobot()  ;

for ii_ = 1:size(worker_robots,2)
    ok = false;
    while ~ok
        x_ = randi(map_x_extent-6)+3  ;
        y_ = randi(map_y_extent-6)+3  ;
        if dist_func(x_,y_) >  6; ok = true ; 
            worker_robots{ii_}.waypoints(:,end+1)=[x_;y_]; 
        end 
    end
    ok2 = false;
    while ~ok2
        try
            x2_ = max(round(randi(round(map_x_extent*2/3)) - map_x_extent/2  )  + x_ , 1);
            y2_ = max(round(randi(round(map_y_extent*2/3)) - map_y_extent/2  ) + y_ , 1) ;
            x2_ = randi(map_x_extent-6)+3  ;
            y2_ = randi(map_y_extent-6)+3  ;
            if dist_func(x2_,y2_) > 6  &&  norm_2( [x2_;y2_]-[x_;y_],1 ) > 12; ok2 = true ;      
                worker_robots{ii_}.waypoints(:,end+1)=[x2_;y2_];    
            end
        catch
        end            
    end
end

%  draw the end points
for ii_ = 1:size(worker_robots,2)
    robot = worker_robots{ii_} ;
    start_point = robot.waypoints(:,1)  ;
    end_point = robot.waypoints(:,2)  ;
    x_ = start_point(1); y_ = start_point(2);   %       [x_ y_]
            hold on;  grid on; plot3(y_,x_,dist_func(x_,y_), 'rx', 'LineWidth',10); 
            text(double(y_),double(x_),double(dist_func(x_,y_))+1.5, int2str(ii_), 'Color','r');       
    x2_ = end_point(1); y2_ = end_point(2);
            hold on;  grid on; plot3(y2_,x2_,dist_func(x2_,y2_), 'mx', 'LineWidth',10);  
            text(double(y2_),double(x2_),double(dist_func(x2_,y2_))+1.5, int2str(ii_), 'Color','m'); 
end


% plan a path 
for ii_ = 1:size(worker_robots,2)
    robot = worker_robots{ii_} ;
    start_point = robot.waypoints(:,1)  ;
    end_point = robot.waypoints(:,2)  ;
    [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_point, end_point)    ;
    [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, end_point)    ;
    
    % draw the path
    for jj_ = 1:size(total_path_start_to_goal__,2)
    plot3(  ...
        total_path_start_to_goal__(2,jj_),  ...
        total_path_start_to_goal__(1,jj_),  ...
        dist_func( total_path_start_to_goal__(1,jj_), total_path_start_to_goal__(2,jj_)   ),  ... 
        'rx' , 'LineWidth', 2)   ;
        daspect([1 1 3])
    end
end


      for ii_ = 1:size(walls,2)
          wall = walls{ii_};
          patch ( ...
            [wall.location(2),  wall.location(2)+size(wall.shape,2)-1 , wall.location(2)+size(wall.shape,2)-1 , wall.location(2)], ...
            [ wall.location(1), wall.location(1),wall.location(1)+size(wall.shape,1)-1 , wall.location(1)+size(wall.shape,1)-1] ,...
            [ 0.5 , 0.5 , 0.5 , 0.5], ...
            'w') ;
          wall = walls{ii_};
          patch ( ...
            [wall.location(2),  wall.location(2)+size(wall.shape,2)-1 , wall.location(2)+size(wall.shape,2)-1 , wall.location(2)], ...
            [ wall.location(1), wall.location(1),wall.location(1)+size(wall.shape,1)-1 , wall.location(1)+size(wall.shape,1)-1] ,...
            [ 1.0 , 1.0 , 1.0 , 1.0], ...
            'w') ;
      end

      

for ii_ = 1:1
    robot = worker_robots{ii_} ;
    start_point = robot.waypoints(:,1)  ;
    end_point = robot.waypoints(:,2)  ;
    [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_1, start_point, end_point)    ;
    [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, end_point)    ;
    
    % draw the path
    for jj_ = 1:size(total_path_start_to_goal__,2)
    plot3(  ...
        total_path_start_to_goal__(2,jj_),  ...
        total_path_start_to_goal__(1,jj_),  ...
        dist_func( total_path_start_to_goal__(1,jj_), total_path_start_to_goal__(2,jj_)   ),  ... 
        'cd' , 'LineWidth', 2)   ;
        view(3)
        daspect([1 1 3])
    end
end      
      
      
end






