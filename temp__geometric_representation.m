addpath( '/mnt/nixbig/ownCloud/project_code/3rd_party/DistBetween2Segment/' )

%%


%%  GEOMETRIC REPRESENTATION --> shortest-path as LINE SEGMENTS 
%{  
Geometric of the field of view:  built-in Matlab functions 
        alphaShape
        boundary
        convhull
%}


%{
    Treate like a NavMesh 
        - graph of nodes, edges give adjaceny, 
        - plan path across them with AStar
        - find shortest paths as ... shortest line between the sides of the regions
%}


%% Get the edges of polyhedra by construction 
    %  TODO : use the 3D camera-ray model to get the cost in each volume 

map_image = zeros(30,200)  ;  % 3m across, 20m long, running left-to-right
floorplan_corridor_corners = [  0,0 ; 30,0 ; 30,200 ; 0,200  ]  ;
figure_named('corridor')  ;  hold on; 
plot2_rows(floorplan_corridor_corners','bo')  ;

FoV_1 = [ 30,50  ;  0,25  ;  0,80  ; 30,55 ]  ;  % trapezoid, wider at the far side.  Points 1 and 4 are the near side 
plot2_rows(FoV_1')  ;

FoV_2 = [ 30,150  ;  0,125  ;  0,180  ; 30,155 ]  ; 
plot2_rows(FoV_2')  ;

Unobserved_1 = [ 0,0 ; 0,25 ; 30,50 ; 30,0 ]  ;
plot2_rows(Unobserved_1')

Unobserved_2 = [ 0,80 ; 30,55 ; 30,150 ; 0,125 ]  ;
plot2_rows(Unobserved_2')

Unobserved_3 = [ 0,180 ; 30,155 ; 30,200 ; 0,200 ]  ;
plot2_rows(Unobserved_3')

%%

%  check the iteratoin of sides 
for ii_ = 1:size(FoV_2,1)
    display(sprintf('ii_ = %i',ii_));
    edge_pt1 = FoV_2(ii_,:)  ;
    if ii_ < size(FoV_2,1)
        edge_pt2 = FoV_2(ii_+1, :)  ;
    else
        edge_pt2 = FoV_2(1,:)  ;
        display(sprintf('edge_pt2 = %f,%f',edge_pt2(1),edge_pt2(2)))
    end
    plot2_rows( [ edge_pt1 ; edge_pt2 ]' )  ;
end

% which point on a FoV edge/side mid-point is nearest some point on a side in the next region? 

FoV1_edge_num = -1  ;
FoV2_edge_num = -1  ;
min_dist = (max(size(map_image))^2)*2  ;
for ii_ = 1:size(FoV_1,1)
    FoV1_edge_pt1 = FoV_1(ii_,:)  ;
    if ii_ < size(FoV_1,1)
        FoV1_edge_pt2 = FoV_1(ii_+1, :)  ;
    else
        FoV1_edge_pt2 = FoV_1(1,:)  ;
    end
    %plot2_rows( [ edge_pt1 ; edge_pt2 ]' )  ;
    for jj_ = 1:size(FoV_2,1)
        FoV2_edge_pt1 = FoV_2(jj_,:)  ;
        if jj_ < size(FoV_2,1)
            FoV2_edge_pt2 = FoV_2(jj_+1, :)  ;
        else
            FoV2_edge_pt2 = FoV_2(1,:)  ;
        end
        
        [line1_exit__, line2_exit__, line_between_veline2_exit__c_, dist__] = ...
            shortestLineBetweenLineSegments( ...
                FoV1_edge_pt1,FoV1_edge_pt1,...
                FoV2_edge_pt1,FoV2_edge_pt2)  ;
        if dist__ < min_dist
            min_dist = dist__  
            FoV1_edge_num = ii_  
            FoV2_edge_num = jj_  
            FoV1_exit = line1_exit__
            FoV2_exit = line2_exit__
        end        
    end    
end
plot2_rows(FoV1_exit(1:2)','rs')
plot2_rows(FoV2_exit(1:2)','rs')


% -- draw the edges --

edge_num = 3
shape_to_draw = FoV_1
for ii_ = 1:size(shape_to_draw,1)
    if ii_ == edge_num
        edge_pt1 = shape_to_draw(ii_,:)  ;
        if ii_ < size(shape_to_draw,1)
            edge_pt2 = shape_to_draw(ii_+1, :)  ;
        else
            edge_pt2 = shape_to_draw(1,:)  ;
        end
        plot2_rows( [ edge_pt1 ; edge_pt2 ]' )  ;
        plot2_rows( [ edge_pt1 ; edge_pt2 ]' , 'cs')  ;
    end
end
edge_num = 1
shape_to_draw = FoV_2
for ii_ = 1:size(shape_to_draw,1)
    if ii_ == edge_num
        edge_pt1 = shape_to_draw(ii_,:)  ;
        if ii_ < size(shape_to_draw,1)
            edge_pt2 = shape_to_draw(ii_+1, :)  ;
        else
            edge_pt2 = shape_to_draw(1,:)  ;
        end
        plot2_rows( [ edge_pt1 ; edge_pt2 ]' )  ;
        plot2_rows( [ edge_pt1 ; edge_pt2 ]' , 'cs')  ;
    end
end






%%  Get the edges of polyhedra as a hull around the points touched by rays.
    %  TODO : use the 3D camera-ray model to get the cost in each volume 

        map = zeros(200,50)  ;
         cost_layer_geo = ones(size(map))  ;
         camera_coords = [ 110 ; 1 ]  ;FoV_1
         camera_angle_degs = 0  ;
         
         FoV_half_angle_degs=60  ;
         vec_length=35 ;
         for vec_length_factor = 1:-0.1:0.3
             vec_length_iteration = ceil( vec_length*vec_length_factor )  ;
             for ray_angle_deg = -FoV_half_angle_degs:1:FoV_half_angle_degs
                 % ray_angle_deg = FoV_half_angle_degs  ;

                 ray_vec = [ zeros(vec_length_iteration,1)  (1:vec_length_iteration)' ]'  ;
                 ray_cells = ceil( rot2( deg2rad(camera_angle_degs) ) * rot2(  deg2rad(ray_angle_deg)  )*ray_vec )  ;
                 ray_cells = ray_cells + repmat( camera_coords , 1 , vec_length_iteration )  ;             
    %              cost_layer_geo(ray_cells) = cost_layer_geo(ray_cells) - 0.1  ;             
                %              figure_named('cost_layer_geo') ; surf(cost_layer_geo)             
                 for ii_ = 1:size(ray_cells,2)  % 4m range            
                    %                      display(ii_)  ;
                    %                      display(ray_cells(:,ii_))
                     %      cost_layer_geo( ray_cells(1,ii_) , ray_cells(2,ii_) ) = cost_layer_geo( ray_cells(1,ii_) , ray_cells(2,ii_) ) - 0.1  ;                  
                     cost_layer_geo( ray_cells(1,ii_) , ray_cells(2,ii_) ) = 0.5*vec_length_factor  ;                  
                 end
             end
         end

figure ;   surf(cost_layer_geo)

[x,y]=find(cost_layer_geo < 1)

%  --  convhull  --  
%       [K,V] = CONVHULL(...) returns the convex hull K and the corresponding 
%       area/volume V bounded by K
[K,V] = convhull(x,y  ,  'simplify', true) ;
figure;  plot( x(K) , y(K) , 'rx')

hold on;   plot(  x(2:3) , y(2:3)  )

for ii_ = 2:size(K,1)
    hold on;   plot(  x(ii_-1:ii_) , y(ii_-1:ii_)  )
end
hold on;  plot( x(K) , y(K) )


%  --  boundary  --
%       K = boundary(...,S) provides an option of specifying the shrink factor S.
%       The scalar S has a value in the range 0<=S<=1. Setting S to 0 gives the
%       convex hull, while setting S to 1 gives a compact boundary that envelops
%       the points. The default shrink factor is 0.5.
figure_named( 'boundary' );
K = boundary(x,y, 0.0)
subplot(2,3, 1);  plot( x(K) , y(K) , 'rx')
K = boundary(x,y, 0.1)
subplot(2,3, 2);  plot( x(K) , y(K) , 'rx')
K = boundary(x,y, 0.2)
subplot(2,3, 3);  plot( x(K) , y(K) , 'rx')
K = boundary(x,y, 0.5)
subplot(2,3, 4);  plot( x(K) , y(K) , 'rx')
K = boundary(x,y, 0.8)
subplot(2,3, 5);  plot( x(K) , y(K) , 'rx')
K = boundary(x,y, 1.0)
subplot(2,3, 6);  plot( x(K) , y(K) , 'rx')


figure;  hold on ;
BW = edge(  cost_layer_geo < 1  ,  'canny'  )  ; % extract edges
[H, THETA, RHO] = hough( BW ) 
imshow(H,[],'XData',THETA,'YData',RHO,'InitialMagnification','fit')  ;
PEAKS  = houghpeaks(H,10,'threshold',ceil(0.3*max(H(:))))  ;
xHough = THETA(PEAKS(:,2)); 
yHough = RHO(PEAKS(:,1));
plot(xHough,yHough,'s','color','white');
lines = houghlines( BW ,  THETA, RHO, PEAKS )
for k = 1:length(lines)
    xy = [lines(k).point1; lines(k).point2];
    plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
end

%  --  activecontour  --
MASK = zeros( size(BW) )  ;
MASK(:, 1) = 1  ;
MASK(:, 50) = 1  ;
MASK(1, :) = 1  ;
MASK(200, :) = 1  ;
segmented_BW = activecontour(BW, MASK)  ;
        %   segments the 2-D grayscale image A into
        %   foreground (object) and background regions using active contour based
        %   segmentation. The output image BW is a binary image where the
        %   foreground is white (logical true) and the background is black (logical
        %   false). MASK is a binary image, same size as A, that specifies the
        %   initial position of the active contour. 


houghlines

%  --  alphaShape  -- 
