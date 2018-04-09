%{

Aimed at robots moving on a floor - may be some 2D motion/data points assumptions

-- Parameters --
camera_K  -  camera intrinsics matrix , 
Radial          - camera distortion parameters, 
Tangental   - camera distortion parameters, 
points_2D   - distorted 2D/image points, 
points_3D_reported  - 3D/world points, 
num_points, 
split_z,        - whether or not to split the input points between groups in Z dimension   
draw_axes_     - whether or not to draw axes on a figure

-- Algorithm --
undistorts the 2D data
splits the data into 2 sets based on 3D point being in the upper or lower set - robot had upper and lower fiducial markers, and this tries to ensure that the dataset is
not planar
commented-out - samples from the sets
re-orders the data
converts RDF to FLU (i.e. typical camera coordinate system, to typical ground robot coordinate system)
 %}
% - 

%  camera pose estimate   is   T_local(camera)_to_world   because
%  local_origin + local_basis_vectors_hom = [  [0 0 0 1]' , eye(4)] = P3_local, and  
% T_local(camera)_to_world * [  [0 0 0 1]' , eye(4)] = T_local(camera)_to_world * local_basis_vectors_hom  = P3_world

%{
Runs EPnP 
- under RANSAC in camera_extrinsics__ransac_over_epnp.m  -  /mnt/nixbig/ownCloud/project_code/camera_extrinsics__ransac_over_epnp.m
- uses  efficient_pnp_will.m  -  /mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/contrib/EPnP/EPnP/efficient_pnp_will.m
    - adpated from  efficient_pnp.m  -  /mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/contrib/EPnP/EPnP/efficient_pnp.m
- converts back from camera-optical coordinate convention (RDF) to robot/UGV coordinate convention (FLU).
%}
function [ ...
        x,y,z, ...
        extrinsic_estimate_as_local_to_world,...
        world_to_local, ...
        est_rotm_flu, ...
        solution_set, Alph] = ...
    camera_extrinsics__epnp ( ...
        camera_K, Radial , Tangental , ...
        points_2D_undistorted, ...
        points_3D, ...
        num_points, ...
        split_z, ...
        draw_axes_ )
addpath('/mnt/nixbig/ownCloud/project_code/')
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

Focal_length =  [camera_K(1,1) , camera_K(2,2)]  ;
Principal_point =  [ camera_K(1,3) , camera_K(2,3) ]  ;

if size(points_3D,1) == 3  ;     if size(points_3D,2) == 3  ;   display('WARNING: points_3d_in is 3x3') ;   end
    points_3D = points_3D';  %     display('efficient_pnp wants row vectors: transposing points_3d_in ')
end
points_3d_in = horzcat(points_3D(1:3,:), ones(size(points_3D,1),1))  ;
if size(points_2D_undistorted,1) == 2;     if size(points_2D_undistorted,2) == 2 ;    display('WARNING: points_2d_in is 2x2') ;   end
    points_2D_undistorted = points_2D_undistorted'; %     display('efficient_pnp wants row vectors: transposing points_2d_in ')
end
%       moved this to 
%       undistort_pinhole(k_radial,p_tangental,Principal_point,Focal_length,Uo,Vo)
[u_undistorted,v_undistorted] = undistort_pinhole(  ...
    Radial , Tangental , Principal_point , Focal_length , ...
    points_2D_undistorted(:,1) , points_2D_undistorted(:,2)   ...
    );
points_2d_in = horzcat( u_undistorted , v_undistorted , ones(size(points_2D_undistorted,1),1) );
              
now_time            = datetime('now');   now_time_str    = datetostr(now_time);   
camera_name = 'cam_605'  ;
data_time_input_string_format='yyyy_mm_dd_HH_MM_SS';   data_time = datenum('2018_02_13_20_00_19',  data_time_input_string_format);
data_time_str   = datetostr(data_time);

%       x3d_h: homogeneous coordinates of the points in world reference - points3D_world
%       x2d_h: homogeneous position of the points in the image plane - points2D_image
%       A: intrinsic camera parameters
%       R: Rotation of the camera system wrt world reference  - RDF - world_to_camera / world_to_local
%       T: Translation of the camera system wrt world reference - RDF - world_to_camera / world_to_local
%       Xc: Position of the points in the camera reference - Xc_local 
% Assumes that the points have been undistorted, I think.
% K camera instrinsics is in standard format 
% indices = [1 5 6 7 10 9];
% indices = 1:size(points_3D_reported,2);
[Re,Te,Xce,best_solutione, solution_set, Alph]=efficient_pnp_will(...
                points_3d_in,...
                points_2d_in,...
                camera_K);             

% [Re,Te,Xce,best_solutione]=efficient_pnp_gauss(...
%                 vertcat(points_3D_reported, ones(1,size(points_3D_reported,2)))',...
%                 vertcat(points_2D,ones(1,size(points_2D,2)))',...
%                 camera_K); 
R = Re; T = Te; Xc = Xce; best_solution = best_solutione;
            
% estimated camera-to-world frame --> estimated world-to-camera frame
world_to_local              =  vertcat(  horzcat(  R,   T  )   ,    [ 0 0 0 1]  ); 
world_to_camera_est =  vertcat(  horzcat( R', (-(R')) * T )  ,   [ 0 0 0 1]  );      

est_rotm_rdf            = world_to_camera_est(1:3,1:3)  ;
rot_flu_maybe =  horzcat(  est_rotm_rdf(:,3) ,  est_rotm_rdf(:,1).*-1 ,  est_rotm_rdf(:,2).*-1  );
quat_flu_maybe = rotm2quat(rot_flu_maybe);

est_rotm_flu_maybe = quat2rotm(quat_flu_maybe);
est_rotm_flu = est_rotm_flu_maybe;    % OUTPUT 
if draw_axes_
    draw_axes_direct( est_rotm_flu_maybe, world_to_camera_est(1:3,4), '', 3, 'bob')
end

%------------
% into xyz euclidean space 
flu_in_flu = [ 
     1  0  0 ; 
     0  1  0 ; 
     0  0  1 ]' ; 
rdf_in_flu = [
     0  0  1 ; 
    -1  0  0 ;
     0 -1  0 ]' ; 

quat_flu_maybe_2222_wxyz  = rotm2quat( world_to_camera_est(1:3,1:3)*rdf_in_flu );
 x=world_to_camera_est(1,4);
 y=world_to_camera_est(2,4);
 z=world_to_camera_est(3,4);
 
 T_FLU_to_RDF = [  % inverse of rdf_in_flu, as a homogeneous transform with no translation
     0    -1     0     0
     0     0    -1     0
     1     0     0     0
     0     0     0     1   ] ;
 extrinsic_estimate_as_local_to_world = [ [ world_to_camera_est(1:3,1:3)*T_FLU_to_RDF(1:3,1:3) , world_to_camera_est(1:3,4)] ; [ 0 0 0 1 ]  ] ;
 
struct_cam_pose_est(1,1).cam              = camera_name;
struct_cam_pose_est(1,1).time               = data_time;
struct_cam_pose_est(1,1).time_str        = data_time_str;
struct_cam_pose_est(1,1).calc_time       = now_time;
struct_cam_pose_est(1,1).calc_time_str =now_time_str;
struct_cam_pose_est(1,1).x=world_to_camera_est(1,4);
struct_cam_pose_est(1,1).y=world_to_camera_est(2,4);
struct_cam_pose_est(1,1).z=world_to_camera_est(3,4);
struct_cam_pose_est(1,1).qx=quat_flu_maybe_2222_wxyz(2);
struct_cam_pose_est(1,1).qy=quat_flu_maybe_2222_wxyz(3);
struct_cam_pose_est(1,1).qz=quat_flu_maybe_2222_wxyz(4);
struct_cam_pose_est(1,1).qw=quat_flu_maybe_2222_wxyz(1);
 if ~exist('cam_pose_estimates','var') 
%     cam_pose_estimates = containers.Map;
    cam_pose_estimates =    struct2table(struct_cam_pose_est);
 else
    cam_pose_estimates =    [  cam_pose_estimates  ;  struct2table(struct_cam_pose_est);  ];
 end
%  cam_pose_estimates('bob') = pose estimate  % map with String index/keys

return; 
end

 