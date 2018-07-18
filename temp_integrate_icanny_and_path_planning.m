addpath('/mnt/nixbig/ownCloud/project_code/')

im_ = imread('/mnt/nixbig/data/project_AA1_backup_pioneer2/2018_01_06_0012_vrep/2018_01_06_0012_vrep/pioneer2/maps/lvl11_map2/cost_map.png')  ;

test_name_what = 'bob'  ;
factor_what = '-9000'  ;
    %   figure_name = strcat(test_name_what,': ',sprintf(' factor %7.f',factor_what))  ;
figure_name = 'figuuuuuuuuuure'  ;

%%

h_fig_map = figure( 'Name','map' )  ;  

cannyed_im = icanny( 100*(im_>180))  ;
%  cannyed_im  =cannyed_im (1:300,1:500) ;  % top left : small, simple section
imshow( cannyed_im )

map_2  = idecimate(cannyed_im , 4)  ;
map_2 = (map_2>0.1)*100000  ;

%  fill in gaps
        % map_2(20 , 112) = 100000 ;
    % kernel_single_pixel_gap = [ ... 
    %    -1 -1 -1 ;
    %     1 -1  1 ;
    %    -1 -1 -1 ]  ;
    % kernel_single_pixel_gap_0 = [ ... 
    %     0  0  0 ;
    %     1 -1  1 ;
    %     0  0  0 ; ]  ;
    % t_gaps_v = conv2( 1.0*(map_2>1), kernel_single_pixel_gap_0'  ,  'same')>1  ;
    % t_gaps= conv2( 1.0*(map_2>1), kernel_single_pixel_gap_0  ,  'same')>1  ;
    % t_gaps_rgb_overlay = cat( 3, map_2 , t_gaps  , t_gaps_v )  ;
    % figure ;  imshow( t_gaps_rgb_overlay )

% closes single-pixel gaps
    %  imshow( cat(  3  ,  1.0*(map_2>1)  ,  zeros(size(map_2))  ,  1.0*(iclose(map_2, ones(3,3))>1)  - 1.0*(map_2>1)  ) )
map_2 = iclose(map_2, ones(3,3))  ;

% closes diagonal 'gaps' - although that might be better done in the A* itself, just do it here.
kernel_single_diagonal_section = [ ...
     -1     1     0
      1    -1    -1
      0    -1    -1  ]  ;
t_gaps_d1 = conv2( 1.0*(map_2>1), kernel_single_diagonal_section  ,  'same')>1  ;  %top-left-bottom-right diagonal: top-left pixel
t_gaps_d2 = conv2( 1.0*(map_2>1), flip( flip( kernel_single_diagonal_section,1) ,2)  ,  'same')>1  ; %top-left-bottom-right diagonal: bottom-right pixel
t_gaps_d3 = conv2( 1.0*(map_2>1),  flip(kernel_single_diagonal_section,2)  ,  'same')>1  ;  %bottom-left top-right diagonal: top-right pixel
t_gaps_d4 = conv2( 1.0*(map_2>1),  flip( kernel_single_diagonal_section,1)  ,  'same')>1  ; %bottom-left top-right diagonal: bottom-left pixel
t_gaps_rgb_overlay = cat( 3, 1.0*(map_2>1) , t_gaps_d1+t_gaps_d2 , t_gaps_d3+t_gaps_d4 )  ;
figure; imshow(t_gaps_rgb_overlay)

t_a = [ ...
     0  0  0  0  0  ;
    -1 -1 -1  1  1 ; 
    -1  1 -1 -1 -1 ;
    -1  1 -1 -1 -1 ;
     0  0  0  0  0  ;
    ] ;
t_gaps_d5 = conv2( 1.0*(map_2>1),  t_a  ,  'same')>2  ;
t_gaps_rgb_overlay = cat( 3, 1.0*(map_2>1) , zeros(size(map_2)),  t_gaps_d5 )  ;
figure; imshow(t_gaps_rgb_overlay)


imshow(map_2); hold on
%%
[ size(cannyed_im)  ;  size(map_2) ]

% start_2 = ceil([ 53 294 ]./4)  ;
start_2 = ceil([ 16 45 ])  ;  % !!!!! is in matrix coordinates: [ y , x ]
% goal_2 = [ 591 995 ]  ;
% goal_2 = ceil([ 288 425 ]./4)  ;
goal_2 = ceil([ 249 171 ])  ;
goal_2 = ceil([ 106 21 ])  ;  % !!!!! is in matrix coordinates: [ y , x ]
goal_2 = ceil([ 171 249 ])  ;  % !!!!! is in matrix coordinates: [ y , x ]
goal_2 = ceil([ 32 216 ])  ;  % !!!!! is in matrix coordinates: [ y , x ]
hold on; plot2_rows( [ start_2(2) ; start_2(1) ] , 'rx' )
hold on; plot2_rows( [ goal_2(2) ; goal_2(1)  ] , 'gx' )

[ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(map_2, start_2, goal_2)    ;
[ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from, goal_2)    ;    

h_fig_came_from = path_planning__draw_came_from(came_from ,  figure_name  )    ;    
%%
figure(h_fig_map);
path_planning__draw_path( total_path_start_to_goal__ )    ;
%%
total_path_smoothed__ = path_planning__smooth_path_lineofsight( total_path_start_to_goal__ , map_2, 'Threshold', 0.1 )    ;

figure(h_fig_map); hold on ;
path_planning__draw_path( total_path_smoothed__ )    ;



