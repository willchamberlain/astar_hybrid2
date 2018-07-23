%% 
addpath( '/mnt/nixbig/ownCloud/project_code/temp__plain_grid_into_map.m' )

%%    Project out a grid 
%           So far mostly just a refresher on linspace and meshgrid and reshape .

x_extent = [1 100]  ; 
y_extent = [1 107]  ;
x_num_cells = x_extent(2)+1-x_extent(1)  ;
y_num_cells = y_extent(2)+1-y_extent(1)  ;
x_sample_spacing = 5  ;
y_sample_spacing = 7  ;
axis_aligned_map_=zeros( x_num_cells , y_num_cells )  ;
axis_aligned_map_(1:5 , 30:100) = 1  ;  % 1 <= y <=5  ; 30 <= x <= 100
axis_aligned_map_(6:30 , 65:85) = 1  ;

figure;  idisp(axis_aligned_map_)

axis_aligned_grid_x = linspace(x_extent(1), x_extent(2), x_num_cells/x_sample_spacing)  ;
axis_aligned_grid_y = linspace(y_extent(1), y_extent(2), y_num_cells/y_sample_spacing)  ;
[X,Y] = meshgrid(axis_aligned_grid_x, axis_aligned_grid_y)  ; 
xy = cat(3,X,Y) ;
xy_rows =  flip(  reshape( xy , [300,2] )'  , 1 ) ;
xy_coords = round(xy)  ;
xy_coords_rows = flip(  reshape( xy_coords , [300,2] )'  , 1 ) ;

hold on;  plot2_rows(   xy_rows  , 'mx')
hold on;  plot2_rows(   xy_coords_rows   , 'cx')


size(axis_aligned_map_)
size(xy_coords_rows)

max(max(xy_coords_rows))
min(min(xy_coords_rows))

max(xy_coords_rows(1,:))   % row 1 are the y coordinates 
max(xy_coords_rows(2,:))   % row 2 are the x coordinates

%  figure ; idisp( axis_aligned_map_(xy_coords_rows(2,:),xy_coords_rows(1,:))  )

size(   axis_aligned_map_(xy_coords_rows(2,:),xy_coords_rows(1,:))   )

%  find(axis_aligned_map_,

for ii_ = 1:300 
    x_coords_ = xy_coords_rows(1,ii_)  ;
    y_coords_ = xy_coords_rows(2,ii_)  ;
    if axis_aligned_map_(y_coords_,x_coords_) 
        hold on;  plot( x_coords_,  y_coords_ , 'rx')  ;
    else
        hold on;  plot( x_coords_,  y_coords_ , 'wx')  ;
    end
end    
figure_named('map'); surf(axis_aligned_map_)  ;
% histogram(axis_aligned_map_,100)  ;

axis_aligned_map_dist = bwdist(axis_aligned_map_,'euclidean')  ;
figure_named('bwdist');   
    subplot(2,2,1); surf(bwdist(axis_aligned_map_,'cityblock'))  ; title('cityblock')  ;
    subplot(2,2,2); surf(bwdist(axis_aligned_map_,'chessboard'))  ;  title('chessboard')  ; 
    subplot(2,2,3); surf(bwdist(axis_aligned_map_,'quasi-euclidean'))  ;  title('quasi-euclidean')  ;
    subplot(2,2,4); surf(bwdist(axis_aligned_map_,'euclidean'))  ;  title( 'euclidean' )  ;

[FX,FY] = gradient(axis_aligned_map_dist)  ; 
figure_named('gradient of bwdist');   
    subplot(2,2,1); surf(bwdist(axis_aligned_map_,'euclidean'))  ; title('bwdist(euclidean)')  ;
    subplot(2,2,2); surf(FX)  ;  title('FX')  ; 
    subplot(2,2,3); surf(FY)  ;  title('FY')  ;
    subplot(2,2,4); surf( sqrt( FX.^2 + FY.^2 ) )  ;  title('kind of the starting shape, not the magnitude as I thought')  ;
    subplot(2,2,4); surf( FX + FY )  ;  title('summed')  ;

    [tx,ty]= meshgrid( 1:107 , 1:100 )   ;
    figure ;   quiver(tx,ty,FX,FY)
    
    diff_ = diff(axis_aligned_map_dist)  ;
    figure_named('diff') ;  surf( diff_ )  ;
    
    diff_2_ = diff(axis_aligned_map_dist,2)  ;
    figure_named('2nd diff') ;  surf( diff_2_ )  ;
    
    
    axis_aligned_map_dist_reciprocal = 1./axis_aligned_map_dist  ;
    [FX_r,FY_r] = gradient(axis_aligned_map_dist_reciprocal)  ; 
    figure_named('reciprocal') ;  
        subplot(1,2,1) ;  surf(axis_aligned_map_dist_reciprocal) ;  title('reciprocal') ;
%         subplot(1,2,2) ;  quiver(tx,ty,abs(FX_r).*1000000,abs(FY_r).*1000000)  ;  title('quiver')  ;
        subplot(1,2,2) ;   quiver(tx,ty,abs(FX_r),abs(FY_r))  ;  title('quiver')  ;
    [FX_r2,FY_r2] = gradient(axis_aligned_map_dist_reciprocal,2)  ; 
    figure_named('reciprocal') ;  
        subplot(1,2,1) ;  surf(axis_aligned_map_dist_reciprocal) ;  title('reciprocal') ;
        
   hold on;  plot( 40,80,'gx' )        
   hold on;  plot( 40+,80,'go' )       
   
   hold on;  plot( 80,40,'bx' )      
   hold on;  plot( ...
       80 + 10000*FX_r(80,40) ,...
       40 + -1*10000*FY_r(80,40) , ...
       'bo')    
   
   hold on;  plot( 75,40,'rx' )      
   hold on;  plot( ...
       75 + 10000*FX_r(75,40) ,...
       40 + -1*10000*FY_r(75,40) , ...
       'ro')
   
   hold on;  plot( 65,40,'rx' )      
   hold on;  plot( ...
       65 + 10000*FX_r(65,40) ,...
       40 + -1*10000*FY_r(65,40) , ...
       'ro')
        
local = axis_aligned_map_dist_reciprocal(  65-3:65+3,40-3:40+3 )
max(max(local))
ind_= (find( local == max(max(local)) ))
local(ind_)
[ local_row,local_col ] = find( local == max(max(local)) )
local(local_row,local_col)
axis_aligned_map_dist_reciprocal(  65,40 )


 quiver(tx,ty,FX,FY)  gives direction  ,  axis_aligned_map_dist_reciprocal gives magnitude 
    
% now can roll the points at the edges of the obstacles downhill  
%   But why ? - simplest thing is just to sample on a reasonable grid size - as we know the robot and may know its motion model/objective - and cull the points
%   that end up intersecting objects 
%  Mind you, nicer to be able to animate the points moving around 

%--------------------------
%% try for a ridgelines implementation, to get a vector away from the obstacles 
% convolve with linear kernels
%
%      Won't work: narrow walls give narrow outputs from the convolution because of limited overlap,
%       and these have low intensity 
axis_aligned_map_2 = axis_aligned_map_ .* 10 ;
axis_aligned_map_2(1:30 , 10:20) = 10  ;
axis_aligned_map_2(30:60 , 10:11) = 10  ;
axis_aligned_map_2(60:90 , 10) = 10  ;
kernel_ = [ 1:5 , 6  , 5:-1:1 ] .* 0.1.*100
kernel_ = ones(1,11) .* 0.1.*100
convolved_ = conv2(axis_aligned_map_2, kernel_) ;
convolved_2_ = conv2(axis_aligned_map_2, kernel_') ;
figure; surf(convolved_)
figure; surf(convolved_2_)


%--------------------------
%% try again for a ridgelines implementation, to get a vector away from the obstacles
%
%  Guided filter with self - should work, but aiming for a convolution (move around narrow offset and multiply with self) over filled self to avoid edge issues: with doing the fill by hand is giving me gip. 
axis_aligned_map_3 = axis_aligned_map_  ;
axis_aligned_map_3_filled_out = zeros( size(axis_aligned_map_3,1)+10+10 , size(axis_aligned_map_3,2)+10+10 )  ;
axis_aligned_map_3_filled_out(  ...
    1+10:size(axis_aligned_map_3,1)+10  , ...
    1+10:size(axis_aligned_map_3,2)+10  ...
    )  =  axis_aligned_map_  ;
axis_aligned_map_3_filled_out(  ...
    1:10  , ...
    1+10:size(axis_aligned_map_3,2)+10 ...
    ) = repmat( axis_aligned_map_3_filled_out(  10+1 , 1+10:size(axis_aligned_map_3,2)+10 ) , [10 , 1])    ;
axis_aligned_map_3_filled_out(  ...
    size(axis_aligned_map_3,1)+1 : size(axis_aligned_map_3,1)+10  , ...
    1+10:size(axis_aligned_map_3,2)+10 ...
    ) = repmat( axis_aligned_map_3_filled_out(  10+1 , 1+10:size(axis_aligned_map_3,2)+10 ) , [10 , 1])    ;
    

figure ;  surf(axis_aligned_map_3_filled_out)
