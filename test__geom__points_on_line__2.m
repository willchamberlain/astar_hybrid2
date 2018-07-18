% relate back to pixel coordinates, to check whether the pixels contain a wall 
%  for e.g. /mnt/nixbig/ownCloud/project_code/path_planning__smooth_path_lineofsight.m 

% longitude = x , latitude = y 
long21 = -1 ; lat11 = -1 ;
long22 = 25 ; lat21 = 3;
long21 = 45 ; lat11 = 50 ;
long22 = 25 ; lat21 = 46;  % x=56 y=39

latdiff = lat21-lat11 ;                  % x2-x1
longdiff = long21-long22 ;          % y1-y2 
num_steps = 5*ceil( max( abs(latdiff), abs(longdiff)) );
n = num_steps ;
longPos = linspace(long21, long22, n)  ;
latPos  = linspace(lat11,  lat21,  n) ;
    x_indices = floor(longPos)  ;
    y_indices = floor(latPos) ; 
    pixel_indices_all = [ x_indices' , y_indices' ]  ;
    pixel_indices = unique(pixel_indices_all,'rows') ;
% ... map_2(y_index, x_index) ...

figure; plot(longPos', latPos', 'Marker','o'); axis equal; grid on; grid minor; 
hold on; plot(longPos'+0.5, latPos'+0.5, 'Marker','s'); 

for ii_=1:num_steps
    display(  [ floor(longPos(ii_)+0.5) , floor(latPos(ii_)+0.5)  ,  ceil(longPos(ii_)+0.5) , ceil(latPos(ii_)+0.5)  ]  )
end

[ floor(longPos) ; floor(longPos)+0.5 ]
[ floor(latPos) ; floor(latPos)+0.5 ]

axes = gca  ;
set(axes, 'MinorGridLineStyle',':')
set(axes, 'XTick',[ floor(min(longPos)-1):1:ceil(max(longPos)+1) ] )
set(axes, 'YTick',[ floor(min(latPos)-1):1:ceil(max(latPos)+1) ] )

figure(h_fig_map) ;  %  floorplan from gmapping in   /mnt/nixbig/ownCloud/project_code/temp_integrate_icanny_and_path_planning.m
hold on; plot(longPos', latPos', 'Marker','o'); axis equal; grid on; grid minor; 
hold on; plot(longPos'+0.5, latPos'+0.5, 'Marker','s'); 

%for ii_=1:num_steps
for ii_=1:size(pixel_indices,1)
    %display(  [ floor(longPos(ii_)+0.5) , floor(latPos(ii_)+0.5)  ,  ceil(longPos(ii_)+0.5) , ceil(latPos(ii_)+0.5)  ]  )
    plot(floor(longPos(ii_))+0.5 ,floor(latPos(ii_))+0.5 , 'cs') 
    plot(floor(longPos(ii_))+0.5 ,floor(latPos(ii_))+0.5 , 'bd') 
    plot(floor(longPos(ii_))+0.5 ,floor(latPos(ii_))+0.5 , 'gx') 
    x_index = floor(longPos(ii_))  ;
    y_index = floor(latPos(ii_)) ; 
    x_index = pixel_indices(ii_,1)  ;
    y_index = pixel_indices(ii_,2)  ;
    patch( [ x_index x_index x_index+1 x_index+1 ] , [ y_index y_index+1 y_index+1 y_index ] , 'y')  ;
    patch( [ x_index-0.5 x_index-0.5 x_index-0.5+1 x_index-0.5+1 ] , [ y_index-0.5 y_index-0.5+1 y_index-0.5+1 y_index-0.5 ] , 'c')  ;
    plot( x_index+0.5 ,y_index+0.5 , 'ks') 
    plot( x_index+0.5 ,y_index+0.5 , 'bx') 
    plot( x_index ,y_index , 'rs') 
    plot( x_index ,y_index , 'mx') 
    display( sprintf('pixel #%5.2f at (x=%5.2f,y=%5.2f) is valued = %5f', ii_, x_index, y_index ,  ...
        map_2(y_index, x_index) ...
        )  );
end

x_index=1; 
y_index=1 ;
h_patch_11_22 = ...% patch of the first pixel/grid cell:  from 1,1 to 2,2   via 2,1 and 1,2
    patch( [ x_index x_index x_index+1 x_index+1 ] , [ y_index y_index+1 y_index+1 y_index ] , 'c')  ;
h__1_5__1_5 = ...  %  x=1.5, y=1.5 
    plot( x_index+0.5 ,y_index+0.5 , 'ks') ;
plot( x_index+0.5 ,y_index+0.5 , 'bx')  ;
h__1__1 = ...  %  x=1, y=1 
    plot( x_index ,y_index , 'rs')  ;
plot( x_index ,y_index , 'mx')  ;
h__2__1 = ...  %  x=2, y=1
    plot( x_index+1 ,y_index , 'rs')  ;
    plot( x_index+1 ,y_index , 'mx') ;
h__1__2 = ...   %  x=1, y=2 
    plot( x_index ,y_index+1 , 'rs') ;
    plot( x_index ,y_index+1 , 'mx') 
h_patch_00_11 = patch( [ x_index x_index x_index-1 x_index-1 ] , [ y_index y_index-1 y_index-1 y_index ] , 'b')  ;

map_2( 30:40 , 50:60 ) > 0


%--------------------------
%% check Matlab's imshow and pixel/cell boundaries

figure; hold on; imshow(im_); hold on; 
patch( [0 1 1 0], [0 0 1 1], 'w' )
patch( [2 1 1 2], [2 2 1 1], 'w' )
patch( [2 3 3 2], [2 2 3 3], 'w' )
plot(0,0,'cx') ; plot(1,1,'mo')
plot(0.5,0.5,'rs')
plot(2,2,'rs')

figure; hold on; idisp(im_); hold on; 
patch( [0 1 1 0], [0 0 1 1], 'w' )
patch( [2 1 1 2], [2 2 1 1], 'w' )
patch( [2 3 3 2], [2 2 3 3], 'w' )
plot(0,0,'cx') ; 
plot(1,1,'mo')
plot(0.5,0.5,'rs')
plot(2,2,'rs')

size(im_)
patch( [0 1 1 0], 733+1-[0 0 1 1], 'w' )
patch( [2 1 1 2], 733+1-[2 2 1 1], 'w' )
patch( [2 3 3 2], 733+1-[2 2 3 3], 'w' )
plot(0,733+1-0,'cx') ; 
plot(1,733+1-1,'mo')
plot(0.5,733+1-0.5,'rs')
plot(2,733+1-2,'rs')



