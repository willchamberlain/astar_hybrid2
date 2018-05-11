% addpath( '/mnt/nixbig/ownCloud/project_code/' )
%{  
Latency sources:  
1)  shutter close to timestamp on the device
Image.getTimeStamp() :  https://developer.android.com/reference/android/media/Image#gettimestamp
    " Get the timestamp associated with this frame.
    The timestamp is measured in nanoseconds, and is normally monotonically increasing. The timestamps for the images from different sources may have different timebases therefore may not be comparable. The specific meaning and timebase of the timestamp depend on the source providing images. See Camera, CameraDevice, MediaPlayer and MediaCodec for more details. 
    "
2)  clock synchronisation : NTP
%}

%%
%--  Camera intrinsics  from  DEFAULT camera  %-- camera parameters / intrinsics :  need this to get the camera_K at zero offset, because moving the camera alters these intrinsics for
%some reason
% camera_default = CentralCamera('default');  move_trans_x = [ 0 0 0]'; camera_default = camera_default.move(  [   [ eye(3), move_trans_x ] ; [ 0 0 0 1 ]   ]  );  % :-- move in world coordinate system ( FLU ) : camera is aligned to 
% [ camera_K_default , Focal_length_default , Principal_point_default ] = camera_extrinsics__camera_intrinsics_from_pctoolkit_camera(camera_default);
[ camera_K_default , Focal_length_default , Principal_point_default ] = camera_extrinsics__camera_intrinsics_from_default_pctoolkit_cam();

%%
display(' Generate the trajectory ')
start_posn = [6 1 0]  ;
% via_posns = [ 6,2,0; 6,4,0; 7,2,0; 4,3,0 ]  ;  %  via points - testing
% via_posns = [ 4,3,0 ]  ;  %  via points  - single line of points   %-  0001 0002 0003 
% via_posns = [ 5,3,0 ;  4,3,0 ]  ;  %  via points  - single line of points   %-  0004   :  6->5->4  1->2->3
via_posns = [ 4,3,0 ] ;  % 0_*
axis_speed_limits = [0.3 0.3 0.3]  ;
time_under_acc = 5  ;
time_step = 0.005;
[ qb, qbd, qbdd ] = mstraj_2(via_posns, axis_speed_limits, [], start_posn, time_step, time_under_acc);
% q = mstraj(via, [2 1 1], [], [4 1 0], 0.05, 1);  %  via points , axis speed limits , t per seg , initial posn , time step , time under acc between segments   - Only one of QDMAX or TSEG should be specified, the other is set to []
% q = mstraj(via, [0.3 0.3 0.3], [], [4 1 0], 0.05, 5);  %  via points , axis speed limits , t per seg , initial posn , time step , time under acc between segments   - Only one of QDMAX or TSEG should be specified, the other is set to []
% eyeball the trajectory:  plot the trajectory x,y,z components independently:  check that it looks reasonable
% figure('Name','Trajectory position components');hold on; grid on; plot(qb(:,1)');  plot(qb(:,2)');  plot(qb(:,3)')  ; legend('x', 'y', 'z');
% figure('Name','Trajectory velocity components');hold on; grid on; plot(qbd(:,1)');  plot(qbd(:,2)');  plot(qbd(:,3)')  ; legend('x', 'y', 'z');
% figure('Name','Trajectory acceleration components');hold on; grid on; plot(qbdd(:,1)');  plot(qbdd(:,2)');  plot(qbdd(:,3)')  ; legend('x', 'y', 'z');

% positions of features on the robot 
feature_1_pose_SE3 = [ eye(3) [ 0 , 0.2 , 0.645 ]' ; [ 0 0 0 1 ] ];
% feature_2_pose_SE3 = [ eye(3) [ 0 , 0.2 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0001
% feature_2_pose_SE3 = [ eye(3) [ 0 , -0.2 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0002
% feature_2_pose_SE3 = [ eye(3) [ 0 , 0.4 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0003, 0004
% feature_2_pose_SE3 = [ eye(3) [ 0 , -0.2 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0005
% feature_2_pose_SE3 = [ eye(3) [ 0 , 0.2 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0_000, 0_001
% feature_2_pose_SE3 = [ eye(3) [ 0.5 , 0.2 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0_002
% feature_2_pose_SE3 = [ eye(3) [ 0 , 0.2      , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0_003
feature_3_pose_SE3 = [ eye(3) [ 0 , 0.645 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0_003
% % % feature_2_pose_SE3 = [ eye(3) [ 0 , 0.2      , 0.05 ]' ; [ 0 0 0 1 ] ];  %-- 0_003_2
% % % % feature_3_pose_SE3 = [ eye(3) [ 0.2 , -0.645 , 0.25 ]' ; [ 0 0 0 1 ] ];  %-- 0_003_2
% % % feature_3_pose_SE3 = [ eye(3) [ 0.2 , -0.645 , 0.8 ]' ; [ 0 0 0 1 ] ];  %-- 0_003_3
% locations of features in the world as the robot moves through its trajectory
feature_1_positions =  qb' + repmat(feature_1_pose_SE3(1:3,4), 1 , size(qb,1) )   ;
feature_2_positions =  qb' + repmat(feature_2_pose_SE3(1:3,4), 1 , size(qb,1) )   ;
feature_3_positions =  qb' + repmat(feature_3_pose_SE3(1:3,4), 1 , size(qb,1) )   ; %-- 0_003
%     figure('Name', 'Robot features and the robot trajectory'); hold on; grid on ; plot3_rows(feature_1_positions,'gx') ;  plot3_rows(feature_2_positions,'cx') ; plot3_rows(qb','bx');   axis equal

%%    
display( 'Set up true and latency feature positions --> 3D data' )
latency_s = 0.05; %-- 0_000-0_003
latency_s = 0.025; %-- 0_003_1
latency_time_steps = 10  ; %-- 0_000-0_003_3 
latency_time_steps = 5  ; %-- 0_000-0_003_4
%--   latency_time_steps = ceil(latency_s/time_step)  ;
num_points = 100 ;    
%  feature 1 - sample from full trajectory --> observations
% num_points_feature_1 = round(randi(num_points*0.5)  + num_points/4.0)   ;
% num_points_feature_1 = 34  ;  %  round(randi(num_points*0.35)  + num_points/6.0)   ; %-- 0_003
% points_3D_f1_indices = randperm(size(feature_1_positions,2) - latency_time_steps, num_points_feature_1)   ; % random distribution
feat_1_lim = size(feature_1_positions,2) - latency_time_steps   ;
% points_3D_f1_indices = [ 1  ceil(feat_1_lim/4) ceil(feat_1_lim/3) ceil(feat_1_lim/2)  ceil(2*feat_1_lim/3)  ceil(3*feat_1_lim/4)  feat_1_lim ]   ;   % even 14-point distribution - 0001,0002,0003,0004
% points_3D_f1_indices = [ round([1:(feat_1_lim/( (num_points/2) -1)):feat_1_lim]) feat_1_lim]   ;  % even variable-size distribution of points
points_3D_f1_indices = [ round([1:(feat_1_lim/( (num_points/3) -1)):feat_1_lim]) feat_1_lim]   ; %-- 0_003 % even variable-size distribution of points
points_3D_f1                = feature_1_positions(: , points_3D_f1_indices )   ;
points_3D_f1_latency = feature_1_positions(: , points_3D_f1_indices+latency_time_steps )   ;
%  feature 2
% num_points_feature_2 = num_points - num_points_feature_1   ;
% num_points_feature_2 = 33  ;  %  round(randi(num_points*0.35)  + num_points/6.0)   ; %-- 0_003
% num_points_feature_3 = 33  ;  %  num_points - (num_points_feature_1+num_points_feature_2)   ; %-- 0_003
% points_3D_f2_indices = randperm(size(feature_2_positions,2) - latency_time_steps, num_points_feature_2)   ; % random distribution
feat_2_lim = feat_1_lim - 500 ;
% points_3D_f2_indices = [ 1  ceil(feat_2_lim/4) ceil(feat_2_lim/3) ceil(feat_2_lim/2)  ceil(2*feat_2_lim/3)  ceil(3*feat_2_lim/4)  feat_2_lim ]   ;    % even 14-point distribution - 0001,0002,0003,0004
% points_3D_f2_indices = [ round([1:(feat_2_lim/( (num_points/2) -1)):feat_2_lim]) feat_2_lim]   ; % even variable-size distribution of points
points_3D_f2_indices = [ round([1:(feat_2_lim/( (num_points/3) -1)):feat_2_lim]) feat_2_lim]   ; %-- 0_003 % even variable-size distribution of points
points_3D_f2                = feature_2_positions(: , points_3D_f2_indices )  ;
points_3D_f2_latency = feature_2_positions(: , points_3D_f2_indices+latency_time_steps )  ;
feat_3_lim = feat_2_lim  ;   %--  0_003 
points_3D_f3_indices = [ round([1:(feat_3_lim/( (num_points/3) -1)):feat_3_lim]) feat_3_lim]   ; %-- 0_003 % even variable-size distribution of points
points_3D_f3                = feature_3_positions(: , points_3D_f3_indices )  ;
points_3D_f3_latency = feature_3_positions(: , points_3D_f3_indices+latency_time_steps )  ;
% {
     fig_3d_handle = figure('Name',strcat(exp_num,' : ','3D scene')); axis equal; grid on; hold on;  xlabel('x'); ylabel('y'); zlabel('z');
     plot3_rows(points_3D_f1,'rx')  ;  plot3_rows(points_3D_f2,'bx')  ;
       plot3_rows(points_3D_f3,'mx')  ;  %-- 0_003
     plot3_rows(points_3D_f1_latency,'ro')  ;  plot3_rows(points_3D_f2_latency,'bo')  ;
     plot3_rows(qb','m')  ;   axis equal   ;
% }

%%%--   3D -  no  latency     
   points_3D_preconditioned_no_latency = [  points_3D_f1  points_3D_f2  points_3D_f3  ]    ;
   %--   3D -  with  latency: no latency set by configuring   latency_time_steps=0 
%    points_3D_preconditioned = [  points_3D_f1_latency  points_3D_f2_latency  ]    ;  %-- 0_000-0_002
   points_3D_preconditioned = [  points_3D_f1_latency  points_3D_f2_latency points_3D_f3_latency ]    ;  %-- 0_003   
   num_points = size(points_3D_preconditioned,2)  ;
   
   fig_3d_handle_check_pre = figure('Name',strcat(exp_num,' : ','check 3D points precond')); axis equal; grid on; hold on;  xlabel('x'); ylabel('y'); zlabel('z');
   plot3_rows(points_3D_preconditioned_no_latency,'rx')
   fig_3d_handle_check_pre_lat = figure('Name',strcat(exp_num,' : ','check 3D points precond lat')); axis equal; grid on; hold on;  xlabel('x'); ylabel('y'); zlabel('z');
   plot3_rows(points_3D_preconditioned_no_latency,'bx')
   
   
    %-- Camera 
    %-- Camera pose setup - place a camera at a random pose 
%     min_angle_degs=45; angle_range_degs=40; x_max=3; y_max=1; z_max=2; proportion_in_fov=1.0;    - pre-testing
%     min_angle_degs=45; angle_range_degs=40; x_max=6; y_max=3; z_max=5; proportion_in_fov=1.0;    -  testing
   min_angle_degs=45; angle_range_degs=40; x_max=6; y_max=3; z_max=5; proportion_in_fov=1.0;    -  0001
% % %     min_angle_degs=45; angle_range_degs=40; x_max=6.5; y_max=3; z_max=2.5; proportion_in_fov=1.0;    % - 0_000-0_003_1
% % %     min_angle_degs=45; angle_range_degs=40; x_max=6.5; y_max=3; z_max=2.5; proportion_in_fov=1.0;    % - 0_000-0_003_1
    if ~exist('camera','var') || (exist('p_change_camera','var') && p_change_camera)
    camera =  camera_extrinsics__place_camera_safely_2( ...
        min_angle_degs,angle_range_degs, ...
        x_max, y_max, z_max, [ 2 2 1 ]' ,  ...   [ 0.5 0.5 0.1]' , ... 
        [ 1.0 1.0 1.0 ]' , points_3D_preconditioned , proportion_in_fov );
    end
    figure(fig_3d_handle) ; hold on;   camera.plot_camera;   hold on;
    draw_axes_direct_c(camera.get_pose_rotation, camera.get_pose_translation, 'true cam', 0.85, 'k' )   % draw the camera pose       
    draw_axes_direct(camera.get_pose_rotation, camera.get_pose_translation, '', 0.75 )   % draw the camera pose        

    %%
    display( 'Try EPnP on the good data or latency data')  %-- see  /mnt/nixbig/ownCloud/project_code/camera_extrinsics__generate_perfect_data.m         
        %--   3D --> 2D  
           points_2D = camera.project( points_3D_preconditioned_no_latency );
           points_2D_preconditioned = points_2D;
           %--   RUN EPNP 
            model_size = 5;
            num_RANSAC_iterations = 1000;   % otten 100-1000, but papers imply can be significantly less 
           [ models , models_max_diff_SE3_element , models_extrinsic_estimate_as_local_to_world ] = ...  % , models_best_solution , models_solution_set ] = ...           
        camera_extrinsics__iterate_epnp  ( ...
            points_2D_preconditioned, points_3D_preconditioned, camera_K_default, ...
            num_RANSAC_iterations, model_size, ...
            camera.get_pose_transform);    

        % fig_3d_handle = gcf 
        camera_extrinsics__plot_3d_estimated_poses   (fig_3d_handle, models_extrinsic_estimate_as_local_to_world)
        plot3( 0 , 0 , 0 , 'bo')
        plot3_rows(points_3D_f2,'go')
        for ii_ = 1:size(points_3D_preconditioned,2) ; text(points_3D_preconditioned(1,ii_),points_3D_preconditioned(2,ii_),points_3D_preconditioned(3,ii_),num2str(ii_));  end
        %   draw_axes_direct(camera.get_pose_rotation, camera.get_pose_translation, '5', 5.0 )   % draw the camera pose      
        %   draw_axes_direct_c(camera.get_pose_rotation, camera.get_pose_translation, '5', 4.0  , 'r' )   % draw the camera pose       
        
 
        fig_handle_2D_no_latency = figure('Name',strcat(exp_num,' : ',sprintf('2D latency_time_steps = %f',latency_time_steps)));  grid on; hold on;
        plot2_rows( [1024 0]' ,'bo')  ;  plot2_rows(points_2D_preconditioned, 'rx')  ; plot2_rows(reshape(camera.limits,2,2),'bo')  ; plot2_rows(reshape(camera.limits,2,2)','bo')  ;        
        axis equal
        for ii_ = 1:size(points_2D_preconditioned,2) ; text(points_2D_preconditioned(1,ii_),points_2D_preconditioned(2,ii_),num2str(ii_));  end
        
    %--   analyse the camera pose estimates 
        %--  position/translation 
    camera_position =   camera.T(1:3,4)   ;
    estimated_positions = models_extrinsic_estimate_as_local_to_world(1:3,4,:)   ;
    estimated_positions = reshape(estimated_positions, [3 num_RANSAC_iterations])   ;
    estimated_position_diffs_1 = diff(   cat( 3, estimated_positions , repmat( camera_position, [1, num_RANSAC_iterations] ) )  ,  1  , 3  )   ;
    
    
    estimated_position_diffs = zeros(size(estimated_positions))   ;
    for ii_ = 1:num_RANSAC_iterations
        estimated_position_diffs(:,ii_) =  estimated_positions(:,ii_) - camera_position   ;
    end
    fig_handle_position_error = figure('Name',strcat(exp_num,' : ','position error'));  fig_handle_position_error.Name='Estimated position errors'  ; grid on; xlabel('estimatenumber'); ylabel('euclidean distance from true camera position');
    posn_euclidean_dist_error = norm_2( estimated_position_diffs, 1)  ;
    hold on; plot(posn_euclidean_dist_error,'bx')  ;
    plot( estimated_position_diffs(1,:) , 'rs' )  ; plot( estimated_position_diffs(2,:) , 'gs' )  ;plot( estimated_position_diffs(3,:) , 'bs' )  ;
    figure('Name',strcat(exp_num,' : ','position errors plotted as 3D points: looking for clusters'))  ;  grid on  ;  hold on  ;  
    plot3_rows(estimated_position_diffs, 'rx')  ;     
    params_as_string = sprintf(' num_points=%d, model_size=%d , num_RANSAC_iterations=%d' , num_points,  model_size , num_RANSAC_iterations ) ;
    figure('Name',  strcat(exp_num,' : ',strcat('Euclidean distance of position errors as hist/density',params_as_string)))  ;  grid on  ;  hold on  ; 
    hist(posn_euclidean_dist_error,100)  ;    
    
    % works, BUT not necessarily useful: could use difference between rays through a plane 
    %  e.g. the floor plane or a robot feature height plane, normalised by the actual camera ray horizontal distance ,
    %  or the distance along the surface of the 5m radius cylinder wall, or the 3D distance between te 5m projection along the real and estimated optic centres
    sorted_eucidean_distance_error=sort(posn_euclidean_dist_error(posn_euclidean_dist_error<1));
    pd = fitdist(sorted_eucidean_distance_error','half normal')  ;
    
    % distribution of reprojection errors 
    
    % { 
    %-- compare the orientations  --  not sure that this is useful  
    model_1_quat = Quaternion( squeeze( models_extrinsic_estimate_as_local_to_world(1:3,1:3,1) ))
    camera.get_pose_rotation
    camera_pose_rotation_quat = Quaternion(camera.get_pose_rotation)
    model_pose_rotation_quat = Quaternion( squeeze( models_extrinsic_estimate_as_local_to_world(1:3,1:3, 99 )) )
    minus(model_pose_rotation_quat,camera_pose_rotation_quat)
    model_pose_rotation_quat * camera_pose_rotation_quat
    model_pose_rotation_quat.inv() * camera_pose_rotation_quat
    diff_quat = model_pose_rotation_quat.inv() * camera_pose_rotation_quat
    % } 
    
    %--  QUESTION: does reprojection error always correspond to Euclidean error? 
    % -- reprojection error  --  is the default that everyone will reach for, and the one that would 
    %       be used for  RANSAC , but _may_ not be the best for trying to understand the/any 
    %       systematic  effects of  noise. latency, etc, on the camera pose estimate. 
    %       Can also exclude the poses precluded by the floorplan. 
    
    points_2D_reprojected = zeros( [ size(points_2D) num_RANSAC_iterations ] )  ;  % 3xnum_datapointsxnum_RANSAC_iterations      
    reprojection_difference = zeros( [ size(points_2D) num_RANSAC_iterations ] )  ;  % 3xnum_datapointsxnum_RANSAC_iterations  
    reprojection_Euclidean = zeros( [ 1 size(points_2D,2) num_RANSAC_iterations ] )  ;  %  1xnum_datapointsxnum_RANSAC_iterations
    reprojection_Euclidean_total = zeros( [ 1 num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
    for ii_ = 1:num_RANSAC_iterations
        pose_estimate = squeeze( models_extrinsic_estimate_as_local_to_world(:,:, ii_) )  ; 
        cam_reproject = CentralCamera('default')  ;  cam_reproject.T = pose_estimate  ;  
%         points_2D_reprojected(:,:,ii_) = cam_reproject.project( points_3D_preconditioned )  ;
        points_2D_reprojected(:,:,ii_) = cam_reproject.project( points_3D_preconditioned_no_latency )  ;        
        reprojection_difference(:,:,ii_) = points_2D_reprojected(:,:,ii_) - points_2D  ;
        reprojection_Euclidean(:,:,ii_) = norm_2(reprojection_difference(:,:,ii_),1)  ;
        reprojection_Euclidean_total(ii_) = sum(reprojection_Euclidean(:,:,ii_))  ;
    end
    figure('Name',strcat(exp_num,' : ','mean reprojection_Euclidean_total per RANSAC iteration'));  hold on  ;
    semilogy(reprojection_Euclidean_total/num_points, 'rx') ;
    semilogy( (reprojection_Euclidean_total.*(reprojection_Euclidean_total>1)/num_points ), 'rs') ;  % higlight the high-magnitude errors 
    hold on; xlabel('iteration'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on;    
    
    figure('Name',strcat(exp_num,' : ','mean reprojection_Euclidean_total per RANSAC iteration'));  hold on  ;
    semilogy(reprojection_Euclidean_total(reprojection_Euclidean_total/num_points < 10)/num_points, 'rx') ;
    % semilogy( (reprojection_Euclidean_total.*(reprojection_Euclidean_total>1)/num_points ), 'rs') ;  % higlight the high-magnitude errors 
    hold on; xlabel('iteration'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on;    
    
    
    mean_reprojection_thresholded = reprojection_Euclidean_total(reprojection_Euclidean_total/num_points < 100)/num_points  ;
    mean_reprojection_thresholded = sort(mean_reprojection_thresholded ) ;
    figure('Name',strcat(exp_num,' : ','mean reprojection error, thresholded - sorted')); plot(mean_reprojection_thresholded)
    figure('Name',strcat(exp_num,' : ','mean reprojection error, thresholded - hist')); hist(mean_reprojection_thresholded,100)   % approximates a half-normal distribution ; very few with almost-zero reprojection, but probably good enough for approx probability distribution and confidence
    
    figure('Name',strcat(exp_num,' : ','pose posn error vs mean reprojection_Euclidean_total'));  
    semilogy( posn_euclidean_dist_error , ...
        reprojection_Euclidean_total/num_points, 'rx') ;
    hold on; xlabel('eucidean_distance_error'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on; 
    
    figure('Name',strcat(exp_num,' : ','pose posn error vs mean mean reprojection_Euclidean_total'));  
    loglog( posn_euclidean_dist_error , ...
        reprojection_Euclidean_total/num_points, 'rx') ;
    hold on; xlabel('eucidean_distance_error'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on; 

    figure(fig_3d_handle) ; 
    
    %--  SAVE THE RESULTS
    display( 'SAVE THE RESULTS')
    save_Results_001(exp_num, description ,  camera ,  qb ,  qbd ,  qbdd ,  start_posn ,  via_posns ,  axis_speed_limits ,  time_under_acc ,  time_step ,  latency_s ,  latency_time_steps ,  num_points ,  feature_1_pose_SE3 ,  feature_1_positions ,  points_3D_f1_indices ,  points_3D_f1 ,  points_3D_f1_latency ,  feature_2_pose_SE3 ,  feature_2_positions ,  points_3D_f2_indices ,  points_3D_f2 ,  points_3D_f2_latency ,  num_RANSAC_iterations ,  models_extrinsic_estimate_as_local_to_world )  ;
    display( 'SAVED RESULTS')
    
        
    %%
    %--   DONE:  Test camera aiming process :  generate one camera pointing at each of the datapoints, project the 3D->2D 
    fig_3d_handle = figure; axis equal; grid on; hold on;  xlabel('x'); ylabel('y'); zlabel('z');
    plot3_rows(points_3D_f1,'rx')  ;  plot3_rows(points_3D_f2,'bx')  ;
    plot3_rows(points_3D_f1_latency,'ro')  ;  plot3_rows(points_3D_f2_latency,'bo')  ;
    plot3_rows(qb','m')  ;   axis equal   ;     
    
    dummy_cam = CentralCamera('Default')  ; 
    cam_array = [dummy_cam]  ;
    
    camera_spatial_distribution_width = 4  ;
    cam_position_offset = mean(points_3D_preconditioned,2) ; 
    for ii_ = 1:num_points
        target_point = points_3D_preconditioned(:,ii_) ;
            cam_position_y = cam_position_offset(2) + (  mod(ii_,camera_spatial_distribution_width)  ) - (  mod(num_points,camera_spatial_distribution_width)  ) ; 
            cam_position_x = cam_position_offset(1) + (  round(ii_/camera_spatial_distribution_width)  )  - (  round(num_points/camera_spatial_distribution_width)  );
            cam_position_z = cam_position_offset(3) + (  2.5  ) ;
            cam_position = [ cam_position_x ; cam_position_y ; cam_position_z ]  ;
        cam_direction_vector = target_point - cam_position ;
            cam_array = [ cam_array CentralCamera('Default') ] ;
            %   camera_rdf_coordinate_system__ = camera_rdf_coordinate_system( vector_along_x_axis_ , vector_in_z_axis_plane_ )
            %       see example in /mnt/nixbig/ownCloud/project_code/camera_rdf_coordinate_system.m 
        camera_rdf_coord_sys = camera_rdf_coordinate_system( cam_direction_vector, [ cam_direction_vector(1:2) ; cam_direction_vector(3)+2 ] )  ;
        cam_T =  rt2tr(  camera_rdf_coord_sys , cam_position)  ;
        cam_array(ii_+1).plot_camera('Tcam',cam_T,'scale', 0.20)  ;        %  note: _try_ this
        draw_axes_direct( camera_rdf_coord_sys,  cam_position, '', 5 )  ;
        drawnow  ; 
        pause 
    end
         
        
        
%%         
%--  Try drawing the camera field of view intersections with the XY plane
plane_xy = [ 1 -1  0 0] ;
rays_at_limits = camera.ray( [0,0 ; 0,1024; 1024,1024 ; 1024,0] )
rays_at_limits = camera.ray( [0,0 ] )
rays_at_limits = camera.ray( [ 0,0 ]' )
rays_at_limits = camera.ray( [0,0 ; 0,1024; 1024,1024 ; 1024,0]' )
rays_at_limits(1).intersect(plane_xy)
rays_at_limits(2).intersect(plane_xy)
rays_at_limits(3).intersect(plane_xy)
rays_at_limits(4).intersect(plane_xy)
camera.centre
plot3( [-2.053886490232917,-2.121026051402937] , [-1.922261054911222,-2.121026051402937] , [1.967118668781156,1.900740029280248] )


%% calc the velocity: not accurate / totally in agreement with mstraj_2()
    diff_between_points = qb(1:end-1,:) - qb(2:end,:)   ;  
    f=@(v)   sqrt(  sum(  v.^2  )  );
    apply_func_to_rows = @(f,M) cell2mat(cellfun(f,num2cell(M,2), 'UniformOutput',0));
    dist_between_points = apply_func_to_rows(f,diff_between_points);    
    vel_between_points = dist_between_points./time_step;

%%  OLD
% construct a coordinate system from a forward vector and zero pitch, zero roll 
    %   Can then use this to construct the robot body frame (base_link pose) at each point 
    %   along the trajectory,  given that the trajectory is given as positions.
    %       Can then use the robot body frames (aka base_link) to get the feature poses.
    %   Can also use this logic to get the robot's frame of current motion from the camera's 
    %   point of view, e.g. to request the robot to move further to the left or right of its current 
    %   motion vector even before establishing the camera location.
    %   NEXT:  don't need this right now. Given the robot trajectory in 5ms (0.005s) time steps,
    %   try the EPnP at [0:5:40]ms latencies --> quick data, quick analysis 
    % circle --> forward vector , then calc y vector, then calc Z as cross product
    %     (x-x_offset)^2 + (y-y_offset)^2 = radius
    %     x^2 + y^2 = 1
    %     y^2 = 1- x^2
    x =  -1:0.1:1     ;
    y = sqrt( ones(size(x)) - x.^2 )  ;
    x = [ x flip(x,2) ] ; 
    y = [ y flip(y,2).*-1 ];
    plot(x,y)  ;
    plot(x,y,'rs')  ;
    plot(x,y.*-1,'bs')  ;
    plot(x,y.*-1,'b')  ;
    %     yaxis_rads = atan2(y,x)   % from here could rotate the origin-to-xaxis_xy vector to get the origin-to-yaxis_xy vector    
    %     figure; axis equal; grid on; hold on    
    left90_SO3 = rotz(90*(2*pi/360))   ; 
    for ii_ = 1:size(x,2) 
        plot3_rows( [ 0 x(1,ii_) ; 0 y(1,ii_) ;  0 , 0 ], 'r')
        plot3_rows( [ 0 x(1,ii_) ; 0 y(1,ii_) ;  0 , 0 ].*1.2, 'r:')
        plot3_rows(left90_SO3*[ 0 x(1,ii_) ; 0 y(1,ii_) ;  0.1 , 0.1 ], 'b')
        z=cross( [ x(1,ii_)  y(1,ii_) 0 ]  ,  left90_SO3*[ x(1,ii_) ; y(1,ii_) ; 0] ) ;
        plot3_rows( [ zeros(3,1) z' ]  );
        pause
    end
    