%{
Working from the camera pose from 2018_02_12 for camera 605
Reprojection walkthrough ; 
    - start with a camera model and known 3D points in the camera
    coordinate frame and plot3 
    - project the points to 2D and plot2 
    - convert the 3D points to a world coordinate frame and plot3
    - put a 3D point into the world coordinate frame and plot3
    - project from the world coordinate frame into the camera and plot2    
Written in sections
%}


%%
% a very good estimate for camera 605 from the setup: very close position,
% close orientation
cam_605_good = [...
    0.887598151824310  -1.97885166499638  0.957895776523009  -0.0318571289684177  0.187864426800859  0.248082165295458  0.949814360672122
    ];

%  camera model  - no distortion 

% pixels 
resolution_v = 355;
resolution_u = 252;

% Camera matrix is set up for a RDF coordinate system.
% Units are pixels 
% ????
camera_K = [...
   322.9596901156589 , 000.0000000000000 , 176.8267919600727 ; %,    //  f_x ,   0 , c_x
   000.0000000000000 , 323.8523693059909 , 146.7681514313797 ; %,    //    0 , f_y , c_y
   0.0 ,                                0.0 ,                                   1.0 ];              
camera_K = [...
   322.9596901156589 , 000.0000000000000 , resolution_v/2 ; %,    //  f_x ,   0 , c_x
   000.0000000000000 , 323.8523693059909 , resolution_u/2 ; %,    //    0 , f_y , c_y
   0.0 ,                                0.0 ,                                   1.0 ];              
camera_K_hom = horzcat( camera_K , [ 0 ; 0 ; 0  ]) 

%%
% Test projection of known 3D points to 2D image plane coordinates, without
%  distortion.

% RDF : forward, right, right, left, left, down , down , up , up , up , up , up , up, up , up
points_3D = [  
    0 0 3 1 ; 
    1 0 3  1;  2 0 3 1 ; 
    -1 0 3 1 ; -2 0 3 1 ; 
    0 1 3 1 ; 0 2 3 1 ; 
    0 -1 3 1 ; 0 -2 3 1 ; 
    0 -1 6 1 ; 0 -2 6 1  ; 0 -1 9 1 ; 0 -2 9 1  ; 0 -1 12 1 ; 0 -2 12 1 ]'

points_3D = [  
    0 0 3 1 ; 
    1 0 3  1;  % 2 0 3 1 ; 
    -1 0 3 1 ; % -2 0 3 1 ; 
    0 1 3 1 ; % 0 2 3 1 ; 
    0 -1 3 1 ; % 0 -2 3 1 ; 
    0 -1 6 1 ; % 0 -2 6 1  ; 
    0 -1 9 1 ; 0 -2 9 1  ; 
    0 -1 12 1 ; %0 -2 12 1 
    ]'
projected_points_2D = camera_K_hom * points_3D
% projected_points_2D(1,:)./projected_points_2D(3,:)
% projected_points_2D(2,:)./projected_points_2D(3,:)
% projected_points_2D(3,:)./projected_points_2D(3,:)
v = @(b) [ b(1,:)./b(3,:) ;  b(2,:)./b(3,:) ; b(3,:)./b(3,:) ]  % define a function: takes an array, does the above with it
projected_points_2D = v(projected_points_2D(:,:))

%    2D   -   
h_fig_2D = figure('Name','2D'); hold on; grid on; xlabel('u'); ylabel('v');axis equal;%
%    NOTE :  inverted --> v is  resolution_v - 
%         --->  set(gca,'Ydir','reverse')
%
set(gca,'Ydir','reverse')
plot(projected_points_2D(1,:) ,projected_points_2D(2,:), 'rx')
for ii_ = 1:size(projected_points_2D,2)
    text( projected_points_2D(1,ii_), projected_points_2D(2,ii_), sprintf('%d',ii_) )
end

%    3D   -   
h_fig_3D = figure('Name','3D'); hold on; grid on; xlabel('x'); ylabel('y');zlabel('z');axis equal;
plot3_rows(points_3D, 'rx')
for ii_ = 1:size(points_3D,2)
    text( points_3D(1,ii_)+0.1, points_3D(2,ii_)+0.1 , points_3D(3,ii_)+0.1 , sprintf('%d',ii_) , 'Color', 'r' );
end


point_3d_1 = [  0 0 3 1 ]'
camera_K_hom * point_3d_1


%%
%  Plot guidelines on the 2D point plot 
figure(h_fig_2D); hold on; 

plot(  [  camera_K_hom(1,3)  ,  camera_K_hom(1,3)  ] , [  camera_K_hom(2,3)-100 , camera_K_hom(2,3)+100  ]  )
plot(  [  camera_K_hom(1,3)-100  ,  camera_K_hom(1,3)+100  ] , [  camera_K_hom(2,3) , camera_K_hom(2,3)  ]  )

plot(  [ 0 0 resolution_v resolution_v 0 ]  ,  [ 0 resolution_u resolution_u 0 0  ]  )
plot(  [ resolution_v/2 , resolution_v/2 ] ,  [ (resolution_u/2)-100 , (resolution_u/2)+100 ] )  
plot(  [ (resolution_v/2)-100 , (resolution_v/2)+100 ] ,  [ (resolution_u/2) , (resolution_u/2) ] )  

optical_axis_offset = [ ...
146.7681514313797 - 252/2  ; 
176.8267919600727 - 355/2  ] ;
%%  
%         3D : show the points_3D from above in world coordinates , then
%         make some world 3D points, project them in the camera 
figure(h_fig_3D); hold on; 

apoint = [2, -1.1, 0.4 ]';
world_to_camera_good_rotm =  quat2rotm( [ cam_605_good(1,7) , cam_605_good(1,4:6) ] );
world_to_camera_good_trans = cam_605_good(1,1:3)';
world_to_camera_good_ = [  ...
    [world_to_camera_good_rotm , world_to_camera_good_trans]  ;
    0 0 0 1 ] 
world_to_local_FLU = [ [ world_to_camera_good_rotm' , world_to_camera_good_rotm' * world_to_camera_good_trans .* -1]  ;  [ 0 0 0 1 ] ]

% points_3D_in_world_RDF = world_to_camera_good_*points_3D 
% figure('Name','Iin_world_RDF'); hold on;grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
% plot3( points_3D_in_world_RDF(1,:)  ,  points_3D_in_world_RDF(2,:)  ,  points_3D_in_world_RDF(3,:), 'rx')
                     
    T_FLU_to_RDF = [ ...
         0    -1     0     0
         0     0    -1     0
         1     0     0     0
         0     0     0     1   ] ;
     forward_RDF =  [  0 0 3 1 ]'  ;
     left_RDF = [  -3 0 0 1 ]'  ;
     up_RDF = [  0 -3 0 1 ]'  ;
     left_up_RDF = [  -3 -3 0 1 ]'  ;
     
     forward_FLU  =  ( forward_RDF' * T_FLU_to_RDF )' 
     left_FLU = (left_RDF' * T_FLU_to_RDF)'
     up_FLU = (up_RDF' * T_FLU_to_RDF)'
     left_up_FLU = (left_up_RDF' * T_FLU_to_RDF)'
          
     forward_FLU_2  =  T_FLU_to_RDF' * forward_RDF
     left_FLU_2 = T_FLU_to_RDF' * left_RDF
     up_FLU_2 = T_FLU_to_RDF' * up_RDF
     left_up_FLU_2 = T_FLU_to_RDF' * left_up_RDF
     
points_3D_in_camera_FLU =  T_FLU_to_RDF' * points_3D    
h_fig_3D_In_camera_FLU =  figure('Name','In_camera_FLU'); hold on;grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
plot3( points_3D_in_camera_FLU(1,:)  ,  points_3D_in_camera_FLU(2,:)  ,  points_3D_in_camera_FLU(3,:), 'rx')
for ii_ = 1:size(points_3D_in_camera_FLU,2)
    text( points_3D_in_camera_FLU(1,ii_)  ,  points_3D_in_camera_FLU(2,ii_)  ,  points_3D_in_camera_FLU(3,ii_) , sprintf('%d',ii_) );
end
     
% 'world_to_camera_est' expresses world coordinates in the camera frame :
% EPnP estimates camera pose in terms of world frame in RDF - I invert that
% to world frame in terms of camera frame, convert to FLU and return it as 
%  'world_to_camera_est'  also  'world_to_camera_good_'
world_to_camera_est_FLU = world_to_camera_good_
points_3D_in_world_FLU = world_to_camera_est_FLU * points_3D_in_camera_FLU
h_fig_3D_In_world_FLU = figure('Name','In_world_FLU'); hold on;grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
plot3( points_3D_in_world_FLU(1,:)  ,  points_3D_in_world_FLU(2,:)  ,  points_3D_in_world_FLU(3,:), 'rx')
for ii_ = 1:size(points_3D_in_world_FLU,2)
    text( points_3D_in_world_FLU(1,ii_)  ,  points_3D_in_world_FLU(2,ii_)  ,  points_3D_in_world_FLU(3,ii_) , sprintf('%d',ii_) );
end
world_to_camera_est_FLU * [ 0 0 0 1]'
draw_axes_direct(   world_to_camera_est_FLU(1:3,1:3),  world_to_camera_est_FLU(1:3,4) ,  'good',  1 )

points_3D_in_world_RDF = world_to_camera_good_*points_3D 

%%
%        Put a point in the _world_, and project it into the camera
%        
points_3D_b_in_world_FLU = [ 4 -1 0.5 1 ; 6 1 0 1  ;  12 1 0 1    ]'

figure(h_fig_3D_In_world_FLU); hold on;
plot3_rows(points_3D_b_in_world_FLU, 'bo')
for ii_ = 1:size(points_3D_b_in_world_FLU,2)
    text( points_3D_b_in_world_FLU(1,ii_)+0.1, points_3D_b_in_world_FLU(2,ii_)+0.1 , points_3D_b_in_world_FLU(3,ii_)+0.1 , sprintf('%d',ii_) , 'Color', 'b' );
end

T_world_to_local_FLU =  tr_invert(world_to_camera_est_FLU)  ;
points_3D_b_in_camera_FLU = T_world_to_local_FLU * points_3D_b_in_world_FLU  ;
points_3D_b_in_camera_RDF = T_FLU_to_RDF * points_3D_b_in_camera_FLU

figure(h_fig_3D); hold on;
plot3_rows(points_3D_b_in_camera_RDF, 'bo')
for ii_ = 1:size(points_3D_b_in_camera_RDF,2)
    text( points_3D_b_in_camera_RDF(1,ii_)+0.1, points_3D_b_in_camera_RDF(2,ii_)+0.1 , points_3D_b_in_camera_RDF(3,ii_)+0.1 , sprintf('%d',ii_) , 'Color', 'b' );
end

figure(h_fig_2D); hold on;
projected_points_2D_b = camera_K_hom *  points_3D_b_in_camera_RDF  ;
projected_points_2D_b = v(projected_points_2D_b(:,:))
plot_rows( projected_points_2D_b , 'bo' ) ;
for ii_ = 1:size(projected_points_2D_b,2)
    text( projected_points_2D_b(1,ii_) + 10, projected_points_2D_b(2,ii_) + 10 , sprintf('%d',ii_) , 'Color', 'b' );
end




%%   do this again with cx,cy in the image centre:  check that the reprojected  points_3D_b  are projected properly  

