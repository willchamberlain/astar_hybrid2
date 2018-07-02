% Want to plan around environment in a sensible way
% Do planning in sim first, then in lab
% Do my own planning to avoid problems with ROS nav stack.

% Hypothesis: given an object - e.g.a wall - bounded by lines
%  1) push the lines out normal to the line away from the object centre by distance=standoff 
%  2) join line ends by arc of circle with radius = standoff
% Probably good enough for now,


room_1 = [ ...  % room vertices : closed room - end vertex==start vertex or need some class of shape
    100 140 ;
    100 160 ;
    170 160 ;
    170 151 ;
    190 151 ;
    190 149 ;
    170 149 ;
    170 140 ;
    100 140 ;
    ] ; 

room_1_b = [ ...  % room vertices : closed room - end vertex==start vertex or need some class of shape
    100 140 ;
    100 150 ; 
        % door here: how?? reductionist: walls->rooms . holist: structure->doors . objectist: door=obj , beam=obj , column\pillar=obj , 
        % ?  gap != obj 
        % ?  OR gap=virtualobj needed/created to close the envelope --> semantic boundary of semantic mapping -->  voxels are FREE or OCCUPIED or P(occupied,z) or P(class,z)
    100 155 ; 
    100 160 ;
    170 160 ;
    170 151 ;
    190 151 ;
    190 149 ;
    170 149 ;
    170 140 ;
    100 140 ;
    ] ; 

wall_room_1_1 = [ ...
    100 140
    100 160
    ] ;


% wall_room_1_1 = [ ... % wall is closed shape between vertices
%     99 140  
%     100 140
%     100 160
%     99 160]  ;

% 1) given vertices (of straight walls), deternine which pixels do they occupy on a floorplan --> use distance function bwdist(image) (Matlab image toolbox) 

% 2) given vertices of straight walls, construct a navmesh:  
%     wall-lines between 
%     sight-lines between vertices
%     sight-lines cannot cross wall-lines





%%

% for 
    
figure     

H = line( [0 10] , [0 10] )    



h__ = animatedline(1:10,1:2:20);
 [x,y] = getpoints(h__)
 H.XData
 
 %%
 
 line_spec = [ 100 140 ;
    100 160 ] 
 Xstart = line_spec(1,1)
 Xend = line_spec(1,2)
 Ystart = line_spec(2,1)
 Yend = line_spec(2,2)
 
 
%%   NEED ONE PER DIAGONAL DIRECTION - integers only, so nice
Xdifference = (Xend-Xstart)
Ydifference = (Yend-Ystart)

Yerror = 0
for x__ =  Xstart : Xend
    plot_pixel(x, y)
    Yerror = Yerror + Ydifference
    if Yerror >= Xdifference 
        y = y + 1
        Yerror = Yerror - Xdifference
    end
end

%% --> /mnt/nixbig/ownCloud/project_code/geom__points_on_line.m
 
 line_spec = [ 100 140 ;
    100.1 140 ] 
 Xstart = line_spec(1,1)
 Ystart = line_spec(1,2)
 Xend = line_spec(2,1)
 Yend = line_spec(2,2)

Xdifference = (Xend-Xstart)
Ydifference = (Yend-Ystart)


figure; hold on; 
if 0 == Xdifference && 0 == Ydifference
        plot([Xstart], [Ystart],'r.')  % plot_pixel(x, y)
elseif Xdifference ~= 0
    y = Ystart
    delta_Y = Ydifference / Xdifference
    for x = Xstart : Xend
        plot([x], [y],'r.')  % plot_pixel(x, y)
        y = y + delta_Y
    end
else % --> Ydifference ~= 0
    x = Xstart
    delta_X = Xdifference / Ydifference
    for y = Ystart : Yend
        plot([x], [y],'r.')  % plot_pixel(x, y)
        x = x + delta_X
    end    
end






