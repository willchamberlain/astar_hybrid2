CentralCamera
%   classdef CentralCamera < Camera
%
%   The camera coordinate system is:
%
%       0------------> u X
%       |
%       |
%       |   + (principal point)
%       |
%       |   Z-axis is into the page.
%       v Y
CentralCamera


%----  do it with a cube : easier to check the projective geometry ----%

cubevertices_x = [1 1 -1 -1]
cubevertices_y = circshift(cubevertices_x,1,2)

square_vertices = [ cubevertices_x ; cubevertices_y ]
square_vertices_to_plot = horzcat(square_vertices,square_vertices(:,1)) 
figure('Name','test:x,y --> square');   hold on;
grid on; axis equal;
plot( square_vertices_to_plot(1,:), square_vertices_to_plot(2,:) )
plot( square_vertices_to_plot(1,:)*2.  , square_vertices_to_plot(2,:)*2. )
%   plottools on
%   plotyy , plotxx
cube_vertices_3x8 = [  [ square_vertices ; -1.*ones(1,4) ]  , [ square_vertices ; 1.*ones(1,4) ]  ]

cube_vertices_3x4x2(:,:,1) = [ square_vertices ; -1.*ones(1,4) ]
cube_vertices_3x4x2(:,:,2) = [ square_vertices ;  1.*ones(1,4) ]

figure('Name','test:  x,y,x --> cube'); hold on; grid on; axis equal;
plot_cuboid (  cube_vertices_3x4x2  ,  cube_vertices_3x8  )

move_trans_z = [ 0 0 -3]'
% move_trans__rdf = flu_to_rdf(move_trans(1:3,:))
move_trans_x = [ -0.3 0 0]'

camera = CentralCamera('default')
camera.centre
camera=camera.move(  [   [ eye(3), move_trans_x ] ; [ 0 0 0 1 ]   ]  )  % :-- move in world coordinate system ( FLU ) : camera is aligned to 
camera.centre
camera.rho
points_2D = camera.project(cube_vertices_3x8)
figure; plot_rows(points_2D(:,1:4),'rx'); hold on; plot_rows(points_2D(:,5:8),'bo'); 
plot_rows( points_2D(:,2) , 'ks')
plot_rows( points_2D(:,6) , 'k*')
plot( 1024 , 1024 , 'bs') ; plot(  0 , 0 , 'bs')
% - works - have geometry --> pixels for a camera with no distortion == undistorted pixels
% NEXT : points_2D_undistorted, points_3D --> EPnP 
points_3D = cube_vertices_3x8
camera_K  =  camera.C
camera_K  =  camera_K(:,1:3)
Focal_length =  [camera_K(1,1) , camera_K(2,2)]  ;
Principal_point =  [ camera_K(1,3) , camera_K(2,3) ]  ;
%       x3d_h: homogeneous coordinates of the points in world reference - points3D_world
%       x2d_h: homogeneous position of the points in the image plane - points2D_image
%       A: intrinsic camera parameters
%       R: Rotation of the camera system wrt world reference  - RDF - world_to_camera / world_to_local
%       T: Translation of the camera system wrt world reference - RDF - world_to_camera / world_to_local
%       Xc: Position of the points in the camera reference - Xc_local 
% Assumes that the points have been undistorted, I think.
% K camera intrinsics is in standard format 
[R,T,Xc,best_solution, solution_set, Alph]=efficient_pnp_will( points_3D' , points_2D' , camera_K );         


%  camera.centre  is  world_coord_frame__to__camera_ccord_frame
%  T  is  camera_ccord_frame__to__world_coord_frame
T - camera.centre

extrinsic_estimate_as_local_to_world =  vertcat(  horzcat( R', (-(R')) * T )  ,   [ 0 0 0 1]  );          
transl(extrinsic_estimate_as_local_to_world) - camera.get_pose_translation
extrinsic_estimate_as_local_to_world - camera.get_pose_transform
max(max(abs(extrinsic_estimate_as_local_to_world - camera.get_pose_transform)))  < 1e-6
1e-6  / 10^-6

% T_FLU_to_RDF = [  % inverse of rdf_in_flu, as a homogeneous transform with no translation
%      0    -1     0     0
%      0     0    -1     0
%      1     0     0     0
%      0     0     0     1   ] ;
% extrinsic_estimate_as_local_to_world = [ [ extrinsic_estimate_as_local_to_world(1:3,1:3)*T_FLU_to_RDF(1:3,1:3) , extrinsic_estimate_as_local_to_world(1:3,4)] ; [ 0 0 0 1 ]  ] 
% extrinsic_estimate_as_local_to_world - camera.get_pose_transform
% max(max(abs(extrinsic_estimate_as_local_to_world - camera.get_pose_transform))) 

camera.perspective  % perspective camera : perspective projection - keeps straight lines straight 
camera.get_pose_transform()
camera.get_pose_rotation()
camera.get_pose_translation()


%  [R,T,Xc,best_solution,opt]=efficient_pnp_gauss(x3d_h,x2d_h,A)
 [R,T,Xc,best_solution,opt]=efficient_pnp_gauss( points_3D' , points_2D' , camera_K ); 

