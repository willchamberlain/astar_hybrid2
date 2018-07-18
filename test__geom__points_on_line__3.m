function vaues_of_cells_on_line__ = test__geom__points_on_line__3(start_2d_vec_xy_, end_2d_vec_xy_ , map_2  )
%  development script: /mnt/nixbig/ownCloud/project_code/test__geom__points_on_line__2.m
% relate back to pixel coordinates, to check whether the pixels contain a wall 
%  for e.g. /mnt/nixbig/ownCloud/project_code/path_planning__smooth_path_lineofsight.m 
%
%  x,y  =  u,v  =  image/pixel coordinates : (1,1) is image top left / floorplan far-west far-north asuming a north-oriented floorplan image/diagram
%  x increases to the right
%  y increases down the image (opposite of plots/charts/graphs) : get(h_figure_of_map,'CurrentAxes').YDir == 'reverse'
%
x_start = start_2d_vec_xy_(1)  ;   y_start=start_2d_vec_xy_(2)  ;
x_end = end_2d_vec_xy_(1)  ;   y_end=end_2d_vec_xy_(2)  ;

y_diff = y_end-y_start ;                  % x2-x1
x_diff = x_start-x_end ;          % y1-y2 
num_steps = 5*ceil( max( abs(y_diff), abs(x_diff)) );
points_x = linspace(x_start, x_end, num_steps)  ;
points_y  = linspace(y_start,  y_end,  num_steps) ;
    x_indices = floor(points_x)  ;
    y_indices = floor(points_y) ; 
    pixel_indices_all = [ x_indices' , y_indices' ]  ;
    pixel_indices = unique(pixel_indices_all,'rows') ;
    num_steps = size(pixel_indices,1)  ;

    %  
    % figure; plot(points_x', points_y', 'Marker','o'); axis equal; grid on; grid minor; 
    % hold on; plot(points_x'+0.5, points_y'+0.5, 'Marker','s'); 
    % axes = gca  ;
    % set(axes, 'MinorGridLineStyle',':')
    % set(axes, 'XTick',[ floor(min(points_x)-1):1:ceil(max(points_x)+1) ] )
    % set(axes, 'YTick',[ floor(min(points_y)-1):1:ceil(max(points_y)+1) ] )
    %  
    % for ii_=1:num_steps
    %     display(  [ floor(points_x(ii_)+0.5) , floor(points_y(ii_)+0.5)  ,  ceil(points_x(ii_)+0.5) , ceil(points_y(ii_)+0.5)  ]  )
    % end
    % [ floor(points_x) ; floor(points_x)+0.5 ]
    % [ floor(points_y) ; floor(points_y)+0.5 ]
    %

    % figure(h_fig_map) ;  %  floorplan from gmapping in   /mnt/nixbig/ownCloud/project_code/temp_integrate_icanny_and_path_planning.m
    % hold on; plot(points_x', points_y', 'Marker','o'); axis equal; grid on; grid minor; 
    % hold on; plot(points_x'+0.5, points_y'+0.5, 'Marker','s'); 

%for ii_=1:num_steps
vaues_of_cells_on_line__ = zeros(1,size(pixel_indices,1)) ;
for ii_=1:size(pixel_indices,1)
    x_index = pixel_indices(ii_,1)  ;
    y_index = pixel_indices(ii_,2)  ;
    vaues_of_cells_on_line__(ii_) = map_2(y_index, x_index)  ;
    %     display(  [ floor(longPos(ii_)+0.5) , floor(latPos(ii_)+0.5)  ,  ceil(longPos(ii_)+0.5) , ceil(latPos(ii_)+0.5)  ]  )
    %     plot(floor(points_x(ii_))+0.5 ,floor(points_y(ii_))+0.5 , 'cs') 
    %     plot(floor(points_x(ii_))+0.5 ,floor(points_y(ii_))+0.5 , 'bd') 
    %     plot(floor(points_x(ii_))+0.5 ,floor(points_y(ii_))+0.5 , 'gx') 
    %     x_index = floor(points_x(ii_))  ;
    %     y_index = floor(points_y(ii_)) ; 
    %     x_index = pixel_indices(ii_,1)  ;
    %     y_index = pixel_indices(ii_,2)  ;
    %     patch( [ x_index x_index x_index+1 x_index+1 ] , [ y_index y_index+1 y_index+1 y_index ] , 'y')  ;
    %     patch( [ x_index-0.5 x_index-0.5 x_index-0.5+1 x_index-0.5+1 ] , [ y_index-0.5 y_index-0.5+1 y_index-0.5+1 y_index-0.5 ] , 'c')  ;
    %     plot( x_index+0.5 ,y_index+0.5 , 'ks') 
    %     plot( x_index+0.5 ,y_index+0.5 , 'bx') 
    %     plot( x_index ,y_index , 'rs') 
    %     plot( x_index ,y_index , 'mx') 
    %     display( sprintf('pixel #%5.2f at (x=%5.2f,y=%5.2f) is valued = %5f', ii_, x_index, y_index ,  ...
    %         map_2(y_index, x_index) ...
    %         )  );
end

end

