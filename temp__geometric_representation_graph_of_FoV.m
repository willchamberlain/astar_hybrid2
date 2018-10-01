%%
addpath( '/mnt/nixbig/ownCloud/project_code/3rd_party/DistBetween2Segment/' )

%%
addpath( '/mnt/nixbig/ownCloud/project_code/')

%%  Simulation - /mnt/nixbig/ownCloud/project_code/vrep_geometric/corridor_2018_09_14_2036.ttt
%{
Treating FoV and unobserved as simple geometric representations.

-- Overlapping FoV --
If FoV edges intersect, aggregate the edges to define one composite FoV/covered region.
Within each covered region plan with AStar.
Between covered regions plan to use shortest path 

-- Gaps around corners --
%}



corridor = [ ...  % per /mnt/nixbig/ownCloud/project_code/vrep_geometric/corridor_2018_09_14_2036.ttt
 7.650, 3.625
-4.325, 3.625
-4.325,-4.450
-1.325,-4.450
-1.325, 0.625
 7.650, 0.625
 ]  ;

corridor = [ ...  % try with some gaps between the camera FoVs and the walls
 7.650, 4.625
-5.325, 4.625
-5.325,-4.450
-0.325,-4.450
-0.325, -0.625
 7.650, -0.625
 ]  ;




cam3_fov = [ ...  (on left around corner)
-4.325,-3.300
-1.325,-2.225
-1.325,-0.125
-4.325, 0.975
]  ;

cam1_fov = [ ... (on straight, nearest corner)
-2.300, 3.625
-1.200, 0.625
 1.025, 0.625
 2.175, 3.625
]  ;

cam2_fov = [ ... (on straight, furthest from corn
%er)
 3.250, 3.625
 4.350, 0.625
 6.550, 0.625
 7.650, 3.650
]  ;
cam_1 = [ ...
    0,0
]  ;
cam_2 = [ ...
    5.5, 0
]  ;
cam_3 = [ ...
    -0.75, -1.1
]  ;

%% draw the map

figure_handle = figure; axis equal; grid on; hold on ; axis equal; grid on; 
patch(corridor(:,1),corridor(:,2),[0.6 1 0.4 ])  ;
patch(cam3_fov(:,1),cam3_fov(:,2),[1 0.4 0.6])  ;
patch(cam1_fov(:,1),cam1_fov(:,2),[1 0.4 0.6])  ;
patch(cam2_fov(:,1),cam2_fov(:,2),[1 0.4 0.6])  ; 
text(cam3_fov(:,1),cam3_fov(:,2),'3')  ;
text(cam1_fov(:,1),cam1_fov(:,2),'1')  ;
text(cam2_fov(:,1),cam2_fov(:,2),'2')  ; 

draw_cam_2d(cam_1(1),cam_1(2),0.3,'black')
 
draw_cam_2d(cam_2(1),cam_2(2),0.3,'k')
 
draw_cam_2d(cam_3(1),cam_3(2),0.3,'k')

%% draw the map

cam_FoVs = cat(3, cam1_fov , cam2_fov , cam3_fov )  ;  %  the set of camera fields of view 
cam_s = cat(3, cam_1 , cam_2 , cam_3)  ;
figure_handle = temp__geometric_representation_graph_of_FoV__draw_map( corridor , cam_FoVs , cam_s )  ; 
% get the start and end points from user input
set(figure_handle,'WindowButtonDownFcn',@callBack_geometric_representation_graph_of_FoV)

%%  Build a graph of FoVs 

% find the nearest FoV to each FoV - checking each border edge of the FoVs
for ii_ = 1:size(cam_FoVs,3)    
    current_min_dist = 10^16  ;
    for jj_ = 1:size(cam_FoVs,3)
        if jj_ == ii_
            display(sprintf('skip:  ii_=%i, jj_=%i', ii_, jj_)) ;  % don't check against itself
            continue
        end
        if jj_ == ii_ ;     display(sprintf('???  SHOULD  have skipped:  ii_=%i, jj_=%i', ii_, jj_)) ;    end
        [ ...
        min_dist, ...
        FoV1_edge_num, FoV2_edge_num, ...
        FoV1_exit, FoV2_exit] ... 
        = shortestLineBetweenPolyhedra(  squeeze(cam_FoVs(:,:,jj_)), squeeze(cam_FoVs(:,:,ii_)) )  ;
        if min_dist < current_min_dist
            current_min_dist = min_dist;                   
            min_distances(ii_).from = ii_;
            min_distances(ii_).to = jj_;
            min_distances(ii_).min_dist = min_dist ;
            min_distances(ii_).FoV1_edge_num = FoV1_edge_num ;
            min_distances(ii_).FoV2_edge_num = FoV2_edge_num ;
            min_distances(ii_).FoV1_exit = FoV1_exit ;
            min_distances(ii_).FoV2_exit = FoV2_exit ;
        end
    end
end

for ii_ = 1:size(min_distances,2)
    min_distances_forward(ii_, :) = [ min_distances(ii_).from , min_distances(ii_).to ]  ;
    min_distances_backward(ii_, :) = [ min_distances(ii_).from , min_distances(ii_).to ]  ;
end



%  e.g.  start in 3, , going to 1, need to search the graph:  follow links from 3-x , x-z , ...  , b-y , y-1
for ii_ = 1:size(min_distances_forward,1)
    s(ii_) = min_distances_forward( ii_ , 1 )  ;
    t(ii_) = min_distances_forward( ii_ , 2 )  ;
end
s 
t
G = digraph(s(2:3),t(2:3));  
figure ; plot(G) ;
sortrows([s;t]')

rows_ = [s;t]'   ;
for ii_ = 1:size(rows_,1) 
    rows_2(ii_,:) = sort( rows_(ii_,:) ) ;
end
rows_2_unique = unique(rows_2,'rows')'
s = rows_2_unique(1,:)
t = rows_2_unique(2,:)
G = graph( s , t );  
figure ; plot(G) ;


%{  
I have the entities, and the graph of FoVs, now how do we transition between the FoVs 
--> Have transition points between each FoV, can run AStar from point-to-point 
First, need to know which FoV we are starting in, and whch FoV the goal is in.
    Note:  in a running system this _could_ come directly from the camera(s) doing the observation
        Note:  if detection in more than one, which one do we plan from/to?
%}

%% get the start and end points from user input
start_point = [6.5,1.25]  ;
start_point = [0,2]  ;

figure_handle = gcf;
set(figure_handle,'WindowButtonDownFcn',@callBack_geometric_representation_graph_of_FoV)
    
%%  Needs USER INPUT  - click on the figure first -  get the start and end points from user input
figure_handle_guidata = guidata(figure_handle) ;
start_point = figure_handle_guidata.start_point(1,:) ; 
plot2_rows(start_point','ro')  ;
text(start_point(1),start_point(2),'Start')  ;
end_point = figure_handle_guidata.start_point(2,:) ;  
plot2_rows(end_point','rs')  ;
text(end_point(1),end_point(2),'End')  ;

%% Work out which FoVs the user clicked in.

fov_in_start = -1;

for ii_ = 1:size(cam_FoVs,3)    
    this_cam = squeeze(cam_FoVs(:,:,ii_))  ;
    [in,on] = inpolygon(start_point(1),start_point(2), this_cam(:,1), this_cam(:,2)) ;
    if in
        fov_in_start = ii_  ;
    end
end

fov_in_end = -1;
for ii_ = 1:size(cam_FoVs,3)    
    this_cam = squeeze(cam_FoVs(:,:,ii_))  ;
    [in,on] = inpolygon(end_point(1),end_point(2), this_cam(:,1), this_cam(:,2)) ;
    if in
        fov_in_end = ii_  ;
    end
end

%%  Have start and end , find shortest path on the graph

path_start_end_fovs = shortestpath(G,fov_in_start,fov_in_end)  ;

% find the way across the gap  --> FoV exit and entry points
num_steps = size(path_start_end_fovs,2)-1  ;
for ii_ = 1:num_steps
    step_start_fov = path_start_end_fovs(ii_)  ;
    step_end_fov = path_start_end_fovs(ii_+1)  ;
     [ ...
        min_dist, ...
        FoV1_edge_num, FoV2_edge_num, ...
        FoV1_exit, FoV2_exit] ... 
        = shortestLineBetweenPolyhedra(  squeeze(cam_FoVs(:,:,step_start_fov)), squeeze(cam_FoVs(:,:,step_end_fov)) )  ;
    
    figure(figure_handle)  ;  hold on  ;
    plot2_rows( [ FoV1_exit(1:2) ; FoV2_exit(1:2) ]' , 'c' , 'Linewidth',6)
    plot2_rows( [ FoV1_exit(1:2) ; FoV2_exit(1:2) ]' , 'm:' , 'Linewidth',6)
end

%%
%{ 
Within each region/between each pair of defined points, 
use AStar with appropriate cost function/map .
    Start-> first FoV exit
    first FoV exit -> second FoV entry 
    second FoV entry -> second FoV exit 
%}
%{
JFDI / spike fast - just join points and move the robot.
%}
waypoints = []  ;
waypoints(1,:) = start_point  ;
num_steps = size(path_start_end_fovs,2)-1  ;
for ii_ = 1:num_steps
    step_start_fov = path_start_end_fovs(ii_)  ;
    step_end_fov = path_start_end_fovs(ii_+1)  ;
     [ ...
        min_dist, ...
        FoV1_edge_num, FoV2_edge_num, ...
        FoV1_exit, FoV2_exit] ... 
        = shortestLineBetweenPolyhedra(  squeeze(cam_FoVs(:,:,step_start_fov)), squeeze(cam_FoVs(:,:,step_end_fov)) )  ;
    waypoints(size(waypoints,1)+1,:) = FoV1_exit(1:2);
    waypoints(size(waypoints,1)+1,:) = FoV2_exit(1:2);
end
waypoints(size(waypoints,1)+1,:) = end_point  ;
waypoints = unique(waypoints,'rows')  ;

figure(figure_handle)  ;  hold on  ;
for ii_ = 1:size(waypoints,1)-1
    plot2_rows(waypoints(ii_:ii_+1,:)', 'b:', 'Linewidth', 3) 
    pause
    display(sprintf('plotted step %i out of %i.',ii_,size(waypoints,1)-1))  ;
end

%%  Quick try to finish this off - Navmesh concave shapes, like gap #2
% First, how can I get the shape of gap #2 ? 
% Gap #2 only exists as the negative of FoV 3 and FoV1 , plus the floorplan 
% I only want the gap shape so that the path can avoid the corner of the corrdior 
%
%{  
    Check for intersection with a constraint (e.g. obstacle, wall) on the floorplan - and its not intersecting at the ends .
    Find the ends of the line constraint: which one is close to the other FoV?
        Closer-to-Fov2  end  :-  is the line _to_ that clear from FoV1_exit ?
            Closer-to-Fov2  end  :-  is the line _from_ that clear to FoV2_entry ?
%}

%   waypoints(ii_,:)  ;
obstacle_edges = corridor   ;

waypoints_adjusted = waypoints ;

for waypoint_num_ = 1:size(waypoints_adjusted,1)-1
    waypoint_pair = waypoints_adjusted(waypoint_num_:waypoint_num_+1,:)  ; 
    display(sprintf('waypoint_num_=%i',waypoint_num_))
    for wall_num_ = 1:size(obstacle_edges,1)-1 
        obstacle_edge = obstacle_edges(wall_num_:wall_num_+1,:)  ;
        [line1_exit__, line2_exit__, line_between_veline2_exit__c_, dist__] = ...
            shortestLineBetweenLineSegments( ...
                waypoint_pair(1,:) , waypoint_pair(2,:) , ...
                obstacle_edge(1,:) , obstacle_edge(2,:) )  ;
        %if dist__ <= 1e-6
            %   display(sprintf( 'intersection! waypoint_num_=%i , walll_num_=%i', waypoint_num_, wall_num_ ))  ;
            line1_exit__ = line1_exit__(1:2);
            line2_exit__ = line2_exit__(1:2);
            if      abs(sum(line1_exit__ - waypoint_pair(1,:) ) ) < 0.01  ...
                ||  abs(sum(line1_exit__ - waypoint_pair(2,:) ) ) < 0.01 ...
                ||  abs(sum(line2_exit__ - waypoint_pair(1,:) ) ) < 0.01  ...
                ||  abs(sum(line2_exit__ - waypoint_pair(2,:) ) ) < 0.01  ...
                ||  abs(sum(line1_exit__ - obstacle_edge(1,:) ) ) < 0.01  ...
                ||  abs(sum(line1_exit__ -  obstacle_edge(2,:) ) ) < 0.01  ...
                ||  abs(sum(line2_exit__ -  obstacle_edge(1,:) ) ) < 0.01  ...
                ||  abs(sum(line2_exit__ -  obstacle_edge(2,:) ) ) < 0.01 
                    %display(sprintf( 'intersection AT END! waypoint_num_=%i , walll_num_=%i', waypoint_num_, wall_num_ ))  ;
            else                    
                    display(sprintf( 'intersection NOT at end! waypoint_num_=%i , walll_num_=%i , dist__=%5.15f', waypoint_num_, wall_num_ , dist__))  ;
                    plot2_rows(    obstacle_edge' , 'k' , 'Linewidth' , 4  )  ;
                    plot2_rows(  waypoints_adjusted(waypoint_num_,:)' , 'ks' , 'Linewidth' , 8  )  ;
                    text(waypoints_adjusted(waypoint_num_,1)-0.25, waypoints_adjusted(waypoint_num_,2)+0.25 , sprintf('%i',waypoint_num_) , 'Color', 'm' ) ;
                    plot2_rows(  waypoints_adjusted(waypoint_num_+1,:)' , 'kd' , 'Linewidth' , 8  )  ;
                    text(waypoints_adjusted(waypoint_num_+1,1)-0.25, waypoints_adjusted(waypoint_num_+1,2)+0.25 , sprintf('+1=%i',waypoint_num_+1) , 'Color', 'm' ) ;
                    
                    %  check the ends                    
                    waypoints_adjusted(waypoint_num_+2:end+1,:) = waypoints_adjusted(waypoint_num_+1:end,:)  ;
                    
                    dist_o1_w2 = norm(obstacle_edge(1,:) - waypoint_pair(2,:),2)waypoints_adjusted
                    dist_o2_w2 = norm(obstacle_edge(2,:) - waypoint_pair(2,:),2)
                    if dist_o1_w2 < dist_o2_w2
                        %  use obstacle vertex 1
                        waypoints_adjusted(waypoint_num_+1,:) = obstacle_edge(1,:)                         
                    else
                        %  use obstacle vertex 2
                        waypoints_adjusted(waypoint_num_+1,:) = obstacle_edge(2,:)                 
                    end
            end
        %else
            %   display(sprintf( 'no intersect - waypoint_num_=%i , walll_num_=%i , dist__=%f', waypoint_num_, wall_num_, dist__ ))  ;
        %end
    end
end


for ii_ = 1:size(waypoints_adjusted,1)-1
    plot2_rows(waypoints_adjusted(ii_:ii_+1,:)', 'r:', 'Linewidth', 3) 
    pause
end

%%
    waypoint_num_ = 2 ;
    waypoint_pair = waypoints(waypoint_num_:waypoint_num_+1,:)  ;     
    wall_num_ = 4  ;
    obstacle_edge = obstacle_edges(wall_num_:wall_num_+1,:)  ;
        [line1_exit__, line2_exit__, line_between_veline2_exit__c_, dist__] = ...
            shortestLineBetweenLineSegments( ...
                waypoint_pair(1,:) , waypoint_pair(2,:) , ...
                obstacle_edge(1,:) , obstacle_edge(2,:) )  
            
plot2_rows(waypoint_pair', 'b')            
plot2_rows(obstacle_edge', 'r')            
            
%%
