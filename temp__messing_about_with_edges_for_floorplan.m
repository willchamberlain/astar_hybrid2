

im_ = imread('/mnt/nixbig/data/project_AA1_backup_pioneer2/2018_01_06_0012_vrep/2018_01_06_0012_vrep/pioneer2/maps/lvl11_map2/cost_map.png')  ;
idisp(im_)
dist_im = bwdist(im_,'euclidean')  ;
idisp(dist_im)
histogram(im_)
size(im_)
im_neg = zeros( size(im_) )
im_neg = zeros( size(im_) )  ;
im_neg( im_ < 25 ) = 1
im_neg( im_ < 25 ) = 1  ;
dist_im = bwdist(im_neg,'euclidean')  ;



dist_im_capped( dist_im > 250 ) = 25  ;
idisp ( dist_im_capped )



im_hard_edged = im_ ;





idisp( kdgauss(2) )

class(  im_hard_edged  )


idisp(  iconv( im_hard_edged , kdgauss(2) )  )
idisp(  iconv( im_hard_edged , kdgauss(2)' )  )

edge_u =   iconv( im_hard_edged , kdgauss(2) )  ;
edge_v =   iconv( im_hard_edged , kdgauss(2)' )  ;

idisp( edge_u ) 
idisp( edge_v ) 


figure;  idisp( edge_u + edge_v )

figure;  idisp( sqrt( edge_u.^2 + edge_v.^2 ) )

figure;  surf( sqrt( edge_u.^2 + edge_v.^2 ) ) 

 [du,dv] = isobel( im_hard_edged, kdgauss(2) )  ;

surf(  abs(  du-edge_v  )  )

histogram(  abs(  du-edge_v  ) )

edge_dirs_diff = du-edge_v  ;
edge_dirs_diff_abs = abs(  du-edge_v  )  ;

idisp( edge_dirs_diff_abs )
histogram( edge_dirs_diff_abs )
edge_dirs_diff_abs_capped = edge_dirs_diff_abs ; 
edge_dirs_diff_abs_capped( edge_dirs_diff_abs > 10 ) = 10   ;
histogram( edge_dirs_diff_abs_capped )
edge_dirs_diff_abs_capped( edge_dirs_diff_abs > 1 ) = 1   ;
edge_dirs_diff_abs_capped( edge_dirs_diff_abs > 25 ) = 25   ;
histogram( edge_dirs_diff_abs_capped )
idisp( edge_dirs_diff_abs_capped ) 

figure;  idisp(  icanny( im_hard_edged, 2 )  )

idisp( irank( im_  , 5 , 1) )         %  median equiv =  irank(  image ,  middle_element_of_3x3 ,  half_width_of_filter  )

 [du2,dv2] = isobel(   irank( im_  , 1 , 2)  , kdgauss(2) )  ;
edge_dirs_diff_abs2 = abs(   du2 - dv2   )  ;
histogram(  edge_dirs_diff_abs2  )
idisp(  edge_dirs_diff_abs2  ) 
 
edges = sqrt( edge_u.^2 + edge_v.^2 )  ;
idisp( edges )
idisp(  irank(  irank( edges  , 5 , 1)  , 5 , 1)  )
histogram(   irank(  irank( edges  , 5 , 1)  , 5 , 1)  )
twice_filtered = irank(  irank( edges  , 5 , 1)  , 5 , 1)  ;
histogram(  twice_filtered(twice_filtered<10)  )
idisp( twice_filtered > 20 )

thrice_filtered = irank( twice_filtered  , 5 , 1)  ;
idisp( twice_filtered - thrice_filtered )
idisp( thrice_filtered > 20 )
figure; idisp( thrice_filtered > 30 )

idisp(  icanny(  edges  )  )
figure; idisp(  icanny(  im_  )  )
icanny_edges =  icanny(  edges  )  ;
icanny_im = icanny(  im_  )  ;
dist_icanny_im =  bwdist( icanny_im )  ;
dist_icanny_im( dist_icanny_im > 30 ) = 30  ;
histogram(  dist_icanny_im  )
idisp(  dist_icanny_im  )
surf( dist_icanny_im )

%% ------------------------
%% USE THIS for the ASTAR map 
im_ = imread('/mnt/nixbig/data/project_AA1_backup_pioneer2/2018_01_06_0012_vrep/2018_01_06_0012_vrep/pioneer2/maps/lvl11_map2/cost_map.png')  ;



idisp(100*(im_>150))

canny_factor_im_threshold=1 ;
canny_factor_std_dev_smooth=1;

h_fig_icanny_test = figure;
h_axis_icanny_test(1) = subplot( 2,2,1) ; imshow( icanny( 100*(im_>50*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))
h_axis_icanny_test(2) = subplot( 2,2,2) ; imshow( icanny( 100*(im_>100*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))
h_axis_icanny_test(3) = subplot( 2,2,3) ; imshow( icanny( 100*(im_>150*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))
h_axis_icanny_test(4) = subplot( 2,2,4) ; imshow( icanny( 100*(im_>200*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))

% h_axis = h_axis_icanny_test(1)
% h_axis.get
% h_axis.get('Position')
% get(h_axis_icanny_test(1))
% get(h_axis_icanny_test(3), 'Position')
% pos = get(h_axis_icanny_test(1), 'Position')
set(h_axis_icanny_test(1), 'Position' , [ 0.01             0.5+0.01 0.5-0.01*1.5    0.5-0.01*1.5 ] )
set(h_axis_icanny_test(2), 'Position' , [ 0.5+0.01/2  0.5+0.01  0.5-0.01*1.5    0.5-0.01*1.5 ] )
set(h_axis_icanny_test(3), 'Position' , [ 0.01              0.01         0.5-0.01*1.5   0.5-0.01*1.5 ] )
set(h_axis_icanny_test(4), 'Position' , [ 0.5+0.01/2    0.01        0.5-0.01*1.5    0.5-0.01*1.5 ] )
set(h_axis_icanny_test(1), 'Position' , [pos(1)-0.06 pos(2)-0.06 ] )

idisp( abs( icanny( 100*(im_>150))  - icanny( 100*(im_>100)) ) )

edges_for_astar = 100*icanny( 100*(im_>200)) ;
histogram(edges_for_astar)
imshow(edges_for_astar)

edges_for_astar = icanny( 100*(im_>200))  > 0 ;
idisp( edges_for_astar > 0 )
idisp(edges_for_astar)
histogram(edges_for_astar)
surf(100.0+ edges_for_astar*100.0)
histogram(100.0+ edges_for_astar*100.0)  % -->  100 or 200 : can now model visibility as subtracting from 100 : normalise so that most_visible_pixel=100, unobserved=0, and subtract from this









