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
addpath( '/mnt/nixbig/ownCloud/project_code/' )


%%
%--  Camera intrinsics  from  DEFAULT camera  %-- camera parameters / intrinsics :  need this to get the camera_K at zero offset, because moving the camera alters these intrinsics for
%  some reason
[ camera_K_default , Focal_length_default , Principal_point_default ] = camera_extrinsics__camera_intrinsics_from_default_pctoolkit_cam();

%%
%  DONT_DRAW = 'DONT_DRAW'
% exp_num='now' ; description=exp_num ;
display(strcat(' Generate the trajectory for exp', exp_num, ': ', description))
time_step = 0.005  ;

axis_speed_limits = [1 1 1]  ;  %-- 0_003_6   :  sqrt(1.4^2 / 2) = 0.989949 ~~ 1.00 :  walking speed is 1.4m/s
time_under_acc = 0.5  ;
start_posn = [ 0.0 , 0.0 , 0.0]  ;
end_posn = [6.0 , 0.0 , 0.0]  ; 
deviate = [ 0.0 , 1.2 , 0.0 ]  ;  
deviate = [ 0.0 , 0.6 , 0.0 ]  ;  
via_posns = [   [ 1 , 0.0 , 0.0].*1+deviate  ; [ 1.0 , 0.0 , 0.0].*2 ; [ 1.0 , 0.0 , 0.0].*3-deviate ; [ 1.0 , 0.0 , 0.0].*4 ; [ 1.0 , 0.0 , 0.0].*5+deviate   ];
% via_posns = [   [ 1 , 0.0 , 0.0]  ; [ 2 , 0 , 0 ] ; [ 3.0 , -1.0 , 0.0] ; [ 4.0 , 0.0 , 0.0]  ; [ 4.0 , 1.0 , 0.0]  ];
via_posns = [ via_posns ; end_posn ]  ;  
%{ 
    eyeball: check:  
    figure('Name','Check: start, via, end positions');
    hold on; grid on;  axis equal; plot3_rows( [start_posn ; via_posns]','ro' ); legend('Start, via, end positions'); 
%}
% generate the trajectory 
[ qb, qbd, qbdd ] = mstraj_2(via_posns, axis_speed_limits, [], start_posn, time_step, time_under_acc)  ;
%      figure('Name','Trajectory velocity');hold on; grid on; plot( norm_2(qbd,2) )   ; legend('ve');

% positions of features on the robot 
feature_1_pose_SE3 = [ eye(3) [ 0 , 0.2 , 0.645 ]' ; [ 0 0 0 1 ] ];
feature_3_pose_SE3 = [ eye(3) [ 0 , 0.645 , 0.25 ]' ; [ 0 0 0 1 ] ];  %
feature_2_pose_SE3 = [ eye(3) [ 0 , 0.2      , 0.25 ]' ; [ 0 0 0 1 ] ];  %
% locations of features in the world as the robot moves through its trajectory
feature_1_positions =  qb' + repmat(feature_1_pose_SE3(1:3,4), 1 , size(qb,1) )   ;
feature_2_positions =  qb' + repmat(feature_2_pose_SE3(1:3,4), 1 , size(qb,1) )   ;
feature_3_positions =  qb' + repmat(feature_3_pose_SE3(1:3,4), 1 , size(qb,1) )   ; %-- 0_003
%{
     figure('Name', 'Robot features and the robot trajectory'); 
    hold on; grid on ; plot3_rows(feature_1_positions,'gx') ;  plot3_rows(feature_2_positions,'cx') ; plot3_rows(qb','bx');   axis equal
%}
display(strcat(' Finished: trajectory generated'));

%%    
display( 'Set up true and latency feature positions --> 3D data' )
latency_time_steps = 5 ; %-- 0_003_16
latency_time_steps = 10 ; %-- 0_003_16
latency_time_steps = 2 ; %-- 0_003_16
latency_time_steps = 5 ; %-- 0_003_16
latency_s = latency_time_steps*time_step  ;
%--   latency_time_steps = ceil(latency_s/time_step)  ;
num_obs = 50 ;  
num_obs = floor(50/3) ;  
feat_1_lim = size(feature_1_positions,2) - latency_time_steps   ;
% points_3D_f1_indices = [ round([1:(feat_1_lim/( (num_obs/3) -1)):feat_1_lim]) feat_1_lim]   ; %-- 0_003 % even variable-size distribution of points
points_3D_f1_indices = sort(randperm(feat_1_lim,num_obs))  ;
points_3D_f1                = feature_1_positions(: , points_3D_f1_indices )   ;
points_3D_f1_latency = feature_1_positions(: , points_3D_f1_indices+latency_time_steps )   ;
%  feature 2
feat_2_lim = feat_1_lim - 100 ;
% points_3D_f2_indices = [ round([1:(feat_2_lim/( (num_obs/3) -1)):feat_2_lim]) feat_2_lim]   ; %-- 0_003 % even variable-size distribution of points
points_3D_f2_indices = sort(randperm(feat_1_lim,num_obs))  ;
points_3D_f2                = feature_2_positions(: , points_3D_f2_indices )  ;
points_3D_f2_latency = feature_2_positions(: , points_3D_f2_indices+latency_time_steps )  ;
feat_3_lim = feat_2_lim  ;   %--  0_003 
% points_3D_f3_indices = [ round([1:(feat_3_lim/( (num_obs/3) -1)):feat_3_lim]) feat_3_lim]   ; %-- 0_003 % even variable-size distribution of points
points_3D_f3_indices = sort(randperm(feat_1_lim,num_obs))  ;
points_3D_f3                = feature_3_positions(: , points_3D_f3_indices )  ;
points_3D_f3_latency = feature_3_positions(: , points_3D_f3_indices+latency_time_steps )  ;

%--   3D -  no  latency     
points_3D_preconditioned_no_latency = [  points_3D_f1  points_3D_f2  points_3D_f3  ]    ;

%--   3D -  with  latency: no latency set by configuring   latency_time_steps=0 
points_3D_preconditioned = [  points_3D_f1_latency  points_3D_f2_latency points_3D_f3_latency ]    ;  %-- 0_003   
points_3D_preconditioned = [  points_3D_f1_latency  points_3D_f2_latency points_3D_f3_latency ]    ;  %-- 0_003   

num_points = size(points_3D_preconditioned,2)  ;
%}

%%
display(  'Run the pose estimation'  )
% {
     fig_3d_handle = figure('Name',strcat(exp_num, time_string_for_figuretitle() ,' : ','3D scene')); axis equal; grid on; hold on;  xlabel('x'); ylabel('y'); zlabel('z');
     plot3_rows(points_3D_f1,'rx')  ;   
     plot3_rows(points_3D_f2,'bx')  ;  
     plot3_rows(points_3D_f3,'mx')  ;  
     %  DONT_TEXT_POINTS_3D = 'DONT_TEXT_POINTS_3D'
     if ~exist( 'DONT_TEXT_POINTS_3D' , 'var' )
       pts_ = points_3D_f1;  for ii_ = 1:size(pts_,2) ;  text(pts_(1,ii_),pts_(2,ii_),pts_(3,ii_),num2str(ii_));  end
       pts_ = points_3D_f2; for ii_ = 1:size(pts_,2) ;  text(pts_(1,ii_),pts_(2,ii_),pts_(3,ii_),num2str(ii_));  end
       pts_ = points_3D_f3;  for ii_ = 1:size(pts_,2) ;  text(pts_(1,ii_),pts_(2,ii_),pts_(3,ii_),num2str(ii_));  end
     end
     plot3_rows(points_3D_f1_latency,'ro')  ;  
     plot3_rows(points_3D_f2_latency,'bo')  ; 
     plot3_rows(points_3D_f3_latency,'mo')  ;
     plot3_rows(qb','m')  ;        
     
     axis equal   ;  legend('points_3D_f1', 'points_3D_f2','points_3D_f3', 'points_3D_f1_latency', 'points_3D_f2_latency', 'points_3D_f3_latency','qb')  ;
% }    
  
    %-- Camera 
    %-- Camera pose setup - place a camera looking at one datapointdescription
    num_camera_posn = 4  ;
    rat_ = (size(points_3D_f1,2)-1) / (num_camera_posn-1)  ; 
    indices_ = floor(  [1:num_camera_posn].*rat_ -  rat_ + 1) ;
    points_2D_preconditioned_in_fov_hist = zeros(num_camera_posn,num_points, 'uint32')  ;
    for ii_ii_ = 1: 1
        target_point3D = points_3D_f1(:,indices_(ii_ii_) )  ;
        target_point3D = points_3D_f1(:,round(size(points_3D_f1,2)/2))  ;
        target_point3D = [ 3.5 ; 0.25 ; 0.645 ]  ;
        % Orient camera orthogonal to the x axis, and zero roll
        %  see camera_extrinsics__place_camera_2.m
        cam_pose_xyz = target_point3D + [ -2.0 -2.2 2.0 ]'  ;  % 3m away, 2m up
        cam_pose_xyz = target_point3D + [ 0.0 -3.0 2.0 ]'  ;  % 3m away, 2m up
        cam_pose_xyz = target_point3D + [ -1.0 -2.2 2.0 ]'  ;  % 3m away, 2m up
        cam_direction_vector = target_point3D - cam_pose_xyz ;
        cam_desired_up_vector = cam_direction_vector + [ 0.0 , 0.0 , 2.0]'  ; % camera vertical in-plane with world vertical
        cam_rdf_coord_sys = camera_rdf_coordinate_system(...
            cam_direction_vector, ...
            cam_desired_up_vector )  ;   %  vector_along_x_axis_ , vector_in_z_axis_plane_ )
        cam_SE3 =  rt2tr(  cam_rdf_coord_sys , cam_pose_xyz)  ;
        figure(fig_3d_handle) ;   draw_axes_direct(cam_SE3(1:3,1:3),cam_SE3(1:3,4), '', 0.5)  ;

%         camera = CentralCamera('default','centre',[512 512],'pose',cam_SE3); 
        camera = CentralCamera_default(cam_SE3) ;
        figure( fig_3d_handle )  ; 
        draw_axes_direct_SE3(camera.T, '', 5)  ;
        text(camera.T(1,4),camera.T(2,4),camera.T(3,4), int2str(ii_ii_))  ;            

    % % %
    display( 'Try EPnP on the good data or latency data')  %-- see  /mnt/nixbig/ownCloud/project_code/camera_extrinsics__generate_perfect_data.m         
        % fig_3d_handle = gcf 
        %--   3D --> 2D  
           points_2D = camera.project( points_3D_preconditioned_no_latency ) ;
           points_2D = camera.project( points_3D_preconditioned ) ;
           points_2D_preconditioned = points_2D;
        
           % add noise to 2D data
            p_pts_2D_noise_magnitude = 2  ;  
            p_pts_2D_noise_mean =  0  ; 
            points_2D_noise_u = camera_extrinsics__generate_noise_for_points_2D( size(points_2D,2) ,  p_pts_2D_noise_magnitude ,  p_pts_2D_noise_mean , 1 )    ;
            points_2D_noise_v = camera_extrinsics__generate_noise_for_points_2D( size(points_2D,2) ,  p_pts_2D_noise_magnitude ,  p_pts_2D_noise_mean , 1 )    ;
            points_2D_preconditioned(1,:) = points_2D_preconditioned(1,:) + points_2D_noise_u ;
            points_2D_preconditioned(2,:) = points_2D_preconditioned(2,:) + points_2D_noise_v ;            

            % {
%                 if 1 == ii_ii_ 
                    fig_handle_2D_no_latency = figure('Name',strcat(exp_num,' : ',sprintf('#%d: 2D latency_time_steps = %f',ii_ii_,latency_time_steps)));  grid on; hold on;
                    plot2_rows(points_2D_preconditioned, 'rx')  ;
                    u_0 = camera.limits(1) ; u_max = camera.limits(2) ; v_0 = camera.limits(3) ; v_max = camera.limits(4)  ;
                    plot2_rows( [ u_0  u_max  ;  v_0  v_max ] ,'bo')  ;
                    plot2_rows( [ u_0  u_max  ;  v_max  v_0  ] ,'bo')  ;
                    axis equal
                    %  DONT_TEXT_POINTS_3D = 'DONT_TEXT_POINTS_3D'
                    if ~exist( 'DONT_TEXT_POINTS_3D' , 'var' )
                        for ii_ = 1:size(points_2D_preconditioned,2) ; text(points_2D_preconditioned(1,ii_),points_2D_preconditioned(2,ii_),num2str(ii_));  end
                    end
%                 end
            % }       
    
            figure(fig_handle_2D_no_latency) ;  plot2_rows(points_2D_preconditioned,'rx')
            points_2D_preconditioned_in_fov_hist(ii_ii_,:)  = ...
                    points_2D_preconditioned(1,:) >= camera.limits(1) ...
                    & points_2D_preconditioned(1,:) <= camera.limits(2) ...                
                    & points_2D_preconditioned(2,:) >= camera.limits(3) ...
                    & points_2D_preconditioned(2,:) <= camera.limits(4)  ;
            points_2D_preconditioned_in_fov = points_2D_preconditioned(:, points_2D_preconditioned_in_fov_hist(ii_ii_,:) > 0)  ;
            points_3D_preconditioned_in_fov = points_3D_preconditioned(:, points_2D_preconditioned_in_fov_hist(ii_ii_,:) > 0)  ;
           %--   RUN EPNP 
            p_model_size = 5;
            p_model_size = 12; %-- 0_003_11
            p_model_size = 24; %-- 0_003_12
            p_model_size = 12; %-- 0_003_13
            p_num_RANSAC_iterations = 5000;   % otten 100-1000, but papers imply can be significantly less 
            p_num_RANSAC_iterations = min(500 , 5*size(points_2D_preconditioned_in_fov,2) )  ;
            p_num_RANSAC_iterations = 1000;  
           [ models , models_max_diff_SE3_element , models_extrinsic_estimate_as_local_to_world ] ...
               = camera_extrinsics__iterate_epnp  ( ...
                    points_2D_preconditioned_in_fov,    points_3D_preconditioned_in_fov  , camera_K_default, ...
                    p_num_RANSAC_iterations, p_model_size, ...
                    camera.get_pose_transform);    

        % fig_3d_handle = gcf 
        if ~exist('DONT_DRAW','var') 
            camera_extrinsics__plot_3d_estimated_poses   (fig_3d_handle, models_extrinsic_estimate_as_local_to_world)        
            figure(fig_3d_handle) ;
            plot3_rows(points_3D_f2,'go')  ;   %         plot3( 0 , 0 , 0 , 'bo')  ;
        end
        figure(fig_3d_handle) ;
        draw_axes_direct(camera.get_pose_rotation, camera.get_pose_translation, '', 5.0 )   % draw the camera pose      
        draw_axes_direct_c(camera.get_pose_rotation, camera.get_pose_translation, '', 4.0  , 'g' )   % draw the camera pose       
        draw_axes_direct(camera.get_pose_rotation, camera.get_pose_translation, '', 3.7 )   % draw the camera pose          
    
        % distribution of reprojection errors 
        %--  QUESTION: does reprojection error always correspond to Euclidean error? 
        % -- reprojection error  --  is the default that everyone will reach for, and the one that would 
        %       be used for  RANSAC , but _may_ not be the best for trying to understand the/any 
        %       systematic  effects of  noise. latency, etc, on the camera pose estimate. 
        %       Can also exclude the poses precluded by the floorplan. 

        points_2D_reprojected = zeros( [ size(points_2D) p_num_RANSAC_iterations ] )  ;  % 2xnum_datapointsxnum_RANSAC_iterations      
        reprojection_Manhattan_pts = zeros( [ size(points_2D) p_num_RANSAC_iterations ] )  ;  % 2xnum_datapointsxnum_RANSAC_iterations  
        reprojection_Euclidean_pts = zeros( [ 1 size(points_2D,2) p_num_RANSAC_iterations ] )  ;  %  1xnum_datapointsxnum_RANSAC_iterations
        reprojection_Euclidean_total = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        reprojection_inliers_num_below_1px = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        reprojection_inliers_num_below_2px = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        reprojection_inliers_num_below_4px = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        reprojection_inliers_num_below_8px = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        reprojection_inliers_num_below_16px = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        mean_reprojection_inliers_threshold = 1 ;
        mean_reprojection_inliers_exclude_above_threshold = zeros( [ 1 p_num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
        for ii_ = 1:p_num_RANSAC_iterations 
            pose_estimate = squeeze( models_extrinsic_estimate_as_local_to_world(:,:, ii_) )  ;
%             cam_reproject = CentralCamera('default')  ;  cam_reproject.T = pose_estimate  ;  
            cam_reproject = CentralCamera_default(pose_estimate) ;
    %         points_2D_reprojected(:,:,ii_) = cam_reproject.project( points_3D_preconditioned )  ;
            points_2D_reprojected(:,:,ii_) = cam_reproject.project( points_3D_preconditioned_no_latency )  ;        
            reprojection_Manhattan_pts(:,:,ii_) = points_2D_reprojected(:,:,ii_) - points_2D  ;
            reprojection_Euclidean_pts(:,:,ii_) = norm_2(reprojection_Manhattan_pts(:,:,ii_),1)  ;
            reprojection_Euclidean_total(ii_) = sum(reprojection_Euclidean_pts(:,:,ii_))  ;
            
            reprojection_inliers_num_below_1px(ii_) = sum(reprojection_Euclidean_pts(:,:,ii_)<=1) ;  % TODO _after_ got distribution sorted - get the best by number of inliers, re-estimate based on the pool of all inliers
            reprojection_inliers_num_below_2px(ii_) = sum(reprojection_Euclidean_pts(:,:,ii_)<=2) ;            
            reprojection_inliers_num_below_4px(ii_) = sum(reprojection_Euclidean_pts(:,:,ii_)<=4) ;            
            reprojection_inliers_num_below_8px(ii_) = sum(reprojection_Euclidean_pts(:,:,ii_)<=8) ;            
            reprojection_inliers_num_below_16px(ii_) = sum(reprojection_Euclidean_pts(:,:,ii_)<=16) ;            
            if sum(reprojection_Euclidean_pts(:,:,ii_)<=mean_reprojection_inliers_threshold) > 0
                mean_reprojection_inliers_exclude_above_threshold(ii_)=mean(reprojection_Euclidean_pts(:,reprojection_Euclidean_pts(:,:,ii_)<=mean_reprojection_inliers_threshold, ii_)) ;
            else
                mean_reprojection_inliers_exclude_above_threshold(ii_)=nan;
            end
        end
        
        reprojection_Euclidean_mean = reprojection_Euclidean_total/num_points  ;
        reprojection_Euclidean_mean = squeeze(reprojection_Euclidean_mean)'  ;
        reprojection_error_threshold = quantile([reprojection_Euclidean_total/num_points]',0.5)  ;
        %  reprojection_error_threshold = quantile([reprojection_Euclidean_total/num_points]',1.0)  ;
        if ~exist('DONT_DRAW','var')
            for ii_ = 1:size(models_extrinsic_estimate_as_local_to_world,3)
                if reprojection_Euclidean_mean(ii_) <= reprojection_error_threshold
                    figure(fig_3d_handle)  ;
                    axes_scale = (reprojection_error_threshold-reprojection_Euclidean_mean(ii_)) * (5/reprojection_error_threshold)  ;
                    draw_axes_direct_c( models_extrinsic_estimate_as_local_to_world(1:3,1:3,ii_) , models_extrinsic_estimate_as_local_to_world(1:3,4,ii_) , '' ,  axes_scale , 'k'    )  ;                    
                end
            end
        end
        
    %--   analyse the camera pose estimates 
        %--  position/translation 
    camera_position =   camera.T(1:3,4)   ;
    estimated_positions = models_extrinsic_estimate_as_local_to_world(1:3,4,:)   ;
    estimated_positions = reshape(estimated_positions, [3 p_num_RANSAC_iterations])   ;
    estimated_position_diffs_1 = diff(   cat( 3, estimated_positions , repmat( camera_position, [1, p_num_RANSAC_iterations] ) )  ,  1  , 3  )   ;
    
    
    estimated_position_diffs = zeros(size(estimated_positions))   ;
    for ii_ = 1:p_num_RANSAC_iterations
        estimated_position_diffs(:,ii_) =  estimated_positions(:,ii_) - camera_position   ;
    end
    
    posn_euclidean_dist_error = norm_2( estimated_position_diffs, 1)  ;
    %
    
    camera_orientation_quat = Quaternion(camera.T(1:3,1:3))  ;
    estimated_orientations_SO3 = models_extrinsic_estimate_as_local_to_world(1:3,1:3,:)   ;    
    estimated_orientations_quat = repmat(Quaternion(),  1, p_num_RANSAC_iterations);    
    estimated_orientation_diffs = zeros(p_num_RANSAC_iterations,1)   ;     
    estimated_orientation_diffs_b = zeros(p_num_RANSAC_iterations,1)   ;     
    estimated_orientation_diffs_c = zeros(p_num_RANSAC_iterations,1)   ;    
    for ii_ = 1:p_num_RANSAC_iterations
        estimated_orientations_quat(ii_) =  Quaternion( estimated_orientations_SO3(:,:,ii_)  )  ;
        estimated_orientation_diff = quaternion_distance(  estimated_orientations_quat(ii_) ,  camera_orientation_quat  );
        estimated_orientation_diffs(ii_) = estimated_orientation_diff  ;
        
        estimated_orientation_diffs_2 = estimated_orientation_diffs  ;
        estimated_orientation_diffs_2(estimated_orientation_diffs_2(:,1)<pi*-1) = estimated_orientation_diffs_2(estimated_orientation_diffs_2(:,1)<pi*-1)+2*pi  ;       
        
        estimated_orientation_diffs_reprojection_Euclidean_totalb(ii_) = quaternion_distance_b(  estimated_orientations_quat(ii_) ,  camera_orientation_quat  );
        estimated_orientation_diffs_c(ii_) = quaternion_distance_c(  estimated_orientations_quat(ii_) ,  camera_orientation_quat  );        
    end
    estimated_orientation_diffs_under_reproj_10 = estimated_orientation_diffs(reprojection_Euclidean_mean <= 20,: )  ;   
    %
        
    estimated_positions = models_extrinsic_estimate_as_local_to_world(1:3,4,:)   ;
    estimated_positions = reshape(estimated_positions, [3 p_num_RANSAC_iterations])   ;
        
    estimated_position_diffs_under_reproj_5 = estimated_positions(:,reprojection_Euclidean_mean <= 5 )  ;   
    estimated_position_diffs_under_reproj_5 =  estimated_position_diffs_under_reproj_5 - repmat(camera_position, 1, size(estimated_position_diffs_under_reproj_5,2))   ;
        
    estimated_position_diffs_under_reproj_20 = estimated_positions(:,reprojection_Euclidean_mean <= 20 )  ;   
    estimated_position_diffs_under_reproj_20 =  estimated_position_diffs_under_reproj_20 - repmat(camera_position, 1, size(estimated_position_diffs_under_reproj_20,2))   ;


    estimated_position_diffs_under_dist = estimated_positions( : , posn_euclidean_dist_error < 0.1 )  ;
    estimated_position_diffs_under_dist =  estimated_position_diffs_under_dist - repmat(camera_position, 1, size(estimated_position_diffs_under_dist,2))   ;
    
if size(estimated_position_diffs_under_reproj_20,2) > 5
     fig_now_ = figure('Name',strcat(exp_num,' : #',sprintf('%d',ii_ii_), sprintf('x=%3.3f, y=%3.3f, z=%3.3f', camera.T(1,4), camera.T(2,4), camera.T(3,4)) ,': error distribution under reprojection 20: '));  
     set(fig_now_, 'Position',  [155   165   655  785] ) ;
     mre_xlim = ceil(max(reprojection_Euclidean_mean) / 100)*100  ;
         if ceil(max(reprojection_Euclidean_mean) / 10) > 10 && mre_xlim>1 ; mre_xlim = ceil(max(reprojection_Euclidean_mean) / 10)*10;  end;
     subplot(3,3,1:3); histogram(reprojection_Euclidean_mean, 100); title('Mean reprojection error'); xlabel(' pixels '); xlim([0 mre_xlim]);     
     ylim_ = [0 350]  ;
     ylim_ = [0 min(p_num_RANSAC_iterations, 100)]  ;
     subplot(3,3,4:6); histogram(estimated_orientation_diffs_under_reproj_10(:,1) );  xlim([0 0.2]);  title({'','Orientation error for mean reprojection error <= 20px'}); xlabel(' quaternion distance '); % ylim(ylim_);    
%      norm_dist_q(ii_ii_) = fitdist(estimated_orientation_diffs_under_reproj_10(1,:)','normal') ;
%      conf_bound_q(ii_ii_,:) = [ norm_dist_q(ii_ii_).mu - norm_dist_q(ii_ii_).sigma*3 , norm_dist_q(ii_ii_).mu + norm_dist_q(ii_ii_).sigma*3  ]    ;
    
     A = histcounts(estimated_position_diffs_under_reproj_20(1,:), 'BinLimits',[-0.2, 0.2]);
     ylim_ = [0 min(min(p_num_RANSAC_iterations, 100), max(max(A),10))]  ;
         subplot(3,3,7); histogram(estimated_position_diffs_under_reproj_20(1,:), 'BinLimits',[-0.2, 0.2]); xlabel(' x(m) '); ylim(ylim_); set(gca,'XTick',[-0.2:0.02:0.2]) ;  set(gca,'XTickLabel', str2mat('-0.2', '', '', '', '', '-0.1', '', '', '', '', '0.0', '', '', '', '', '0.1', '', '', '', '', '0.2' )) ;
         subplot(3,3,8); histogram(estimated_position_diffs_under_reproj_20(2,:), 'BinLimits',[-0.2, 0.2]); xlabel(' y(m) '); ylim(ylim_); set(gca,'XTick',[-0.2:0.02:0.2]) ;  set(gca,'XTickLabel', str2mat('-0.2', '', '', '', '', '-0.1', '', '', '', '', '0.0', '', '', '', '', '0.1', '', '', '', '', '0.2' )) ; 
         title({'','Position error for mean reprojection error <= 20px'}); 
         subplot(3,3,9); histogram(estimated_position_diffs_under_reproj_20(3,:), 'BinLimits',[-0.2, 0.2]); xlabel(' z(m) '); ylim(ylim_); set(gca,'XTick',[-0.2:0.02:0.2]) ;  set(gca,'XTickLabel', str2mat('-0.2', '', '', '', '', '-0.1', '', '', '', '', '0.0', '', '', '', '', '0.1', '', '', '', '', '0.2' )) ;             
    norm_dist_x(ii_ii_) = fitdist(estimated_position_diffs_under_reproj_20(1,:)','normal') ;
    conf_bound_x(ii_ii_,:) = [ norm_dist_x(ii_ii_).mu - norm_dist_x(ii_ii_).sigma*3 , norm_dist_x(ii_ii_).mu + norm_dist_x(ii_ii_).sigma*3  ]    ;
    norm_dist_y(ii_ii_) = fitdist(estimated_position_diffs_under_reproj_20(2,:)','normal') ;
    conf_bound_y(ii_ii_,:) = [ norm_dist_y(ii_ii_).mu - norm_dist_y(ii_ii_).sigma*3 , norm_dist_y(ii_ii_).mu + norm_dist_y(ii_ii_).sigma*3  ] ;
    norm_dist_z(ii_ii_) = fitdist(estimated_position_diffs_under_reproj_20(3,:)','normal') ;
    conf_bound_z(ii_ii_,:) = [ norm_dist_z(ii_ii_).mu - norm_dist_z(ii_ii_).sigma*3 , norm_dist_z(ii_ii_).mu + norm_dist_z(ii_ii_).sigma*3  ] ;
end

    %-- compare the orientations  --  not sure that this is useful  
    camera_pose_rotation_quat = Quaternion(camera.get_pose_rotation)  ;
    model_1_quat = Quaternion( squeeze( models_extrinsic_estimate_as_local_to_world(1:3,1:3,1) ))  ;
    model_1_quat * camera_pose_rotation_quat  ;
    model_1_quat.inv() * camera_pose_rotation_quat  ;
    minus(model_1_quat,camera_pose_rotation_quat)
    model_pose_rotation_quat = Quaternion( squeeze( models_extrinsic_estimate_as_local_to_world(1:3,1:3, : )) )  ;
     diff_quat = Quaternion(repmat(ones(3,3), [1, 1 ,p_num_RANSAC_iterations])) ;
    for ii_ = 1 : p_num_RANSAC_iterations
        diff_quat(ii_) = model_pose_rotation_quat(ii_).inv() * camera_pose_rotation_quat ;
    end
    
    mean_reprojection_thresholded = reprojection_Euclidean_total(reprojection_Euclidean_total/num_points < 100)/num_points  ;
    mean_reprojection_thresholded = sort(mean_reprojection_thresholded ) ;
    
    %--  SAVE THE RESULTS
    if ~exist('DONT_SAVE_THE_RESULTS','var')
        display( 'SAVE THE RESULTS')
        save_Results_001(exp_num, description ,  camera ,  qb ,  qbd ,  qbdd ,  start_posn ,  via_posns ,  axis_speed_limits ,  time_under_acc ,  time_step ,  latency_s ,  latency_time_steps ,  num_points ,  feature_1_pose_SE3 ,  feature_1_positions ,  points_3D_f1_indices ,  points_3D_f1 ,  points_3D_f1_latency ,  feature_2_pose_SE3 ,  feature_2_positions ,  points_3D_f2_indices ,  points_3D_f2 ,  points_3D_f2_latency ,  p_num_RANSAC_iterations ,  models_extrinsic_estimate_as_local_to_world )  ;
        display( 'SAVED RESULTS')
    else
        display('Did NOT save the results: DONT_SAVE_THE_RESULTS is set.');
    end
    
    end
    
    
    %%      drop points in, and path plan on them :
%{
--- Version 1: 
    take the best estimate
        Version 1.1: take the best estimate by smallest total reprojection error 
        NEXT Version 1.2: take the best by RANSAC criteria - maximum # inliers +- threshold , re-model on the union of those sets 
    project the pixel-centres of the image onto the world - CentralCamera.ray( [i_u,i_v])-->Ray3D with direction and point in camera coordinates
    project these into the floor plane --> potentially_observale_flooplan
    subtract floorplan obstacles --> potentially_observale_flooplan_2
    volume of robot features that could be observed
        Minkowski sum of robot motion model and robot radius and potentially_observale_flooplan_2
    check the line from camera_centre_on_floor to pixel_ray_on_floor: if it passes through an obstacle before it hits the robot_feature_observable_volume, it is not useful
        NEXT Version 1.2 check for intersection with the actual volume/voxels --> deal with partial cover and low walls
    project the optical axis to the floor plane and find the orthogonal in the floor plane
    those are the axes for a grid of points
        from camera to the FoV intersection with the floor plane 
        Version 1.2: scale the grid to the estimated FoV width 
        
%}
    best_model_by_total_reprojection_error =  find( reprojection_Euclidean_total <= min(reprojection_Euclidean_total) )  ;
    best_model_by_consensus_size = find( reprojection_inliers_num_below_8px >= max(reprojection_inliers_num_below_8px) );
    best_model_id = intersect(best_model_by_total_reprojection_error,best_model_by_consensus_size) ;
    best_pose = models_extrinsic_estimate_as_local_to_world(:,:, best_model_id)  ;
    
    axes_scale = 1;
        draw_axes_direct_c( models_extrinsic_estimate_as_local_to_world(1:3,1:3,best_model_id) , models_extrinsic_estimate_as_local_to_world(1:3,4,best_model_id) , '' ,  axes_scale , 'k'    )  ;                    
        zlim( [-0.2 3.5] )
             
    best_pose_camera = CentralCamera_default( best_pose ) ;
    dummy_ray = Ray3D( [0,0] , [1 1 1] )  ;
    row_num_ = 0  ;
    col_num_ = 0  ;
    uu_step_ = 10  ;
    vv_step_ = 10  ;
    uu_num_rows_ = floor( best_pose_camera.limits(2) / uu_step_ )  ;
    vv_num_cols_ = floor( best_pose_camera.limits(4) / vv_step_ )  ;
    ray_stack_ = repmat( dummy_ray , uu_num_rows_+1 , vv_num_cols_+1 )  ;
    ray_stack_d_ = zeros( uu_num_rows_+1 , vv_num_cols_+1 , 3 )  ;
    ray_stack_P0_ = zeros( uu_num_rows_+1 , vv_num_cols_+1 , 3 )  ;
    z_eq_0_intercept = zeros( uu_num_rows_+1 , vv_num_cols_+1 , 3 )  ;
    scaling_factor_for_vector_along_line_to_intercept = zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;
    z_eq_2_intercept = zeros( uu_num_rows_+1 , vv_num_cols_+1 , 3 )  ;
    scaling_factor_for_vector_along_line_to_intercept_2 = zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;
    % for each pixel in the camera image ...
    for uu_ = 0 : uu_step_ : uu_num_rows_*uu_step_
        row_num_ = row_num_ + 1 ; 
        col_num_ = 0  ;
        for vv_ = 0 : vv_step_ : vv_num_cols_*vv_step_ 
            col_num_ = col_num_ + 1 ;                
            ray_  = best_pose_camera.ray( [ uu_ ; vv_ ] )  ;
            ray_stack_(row_num_,col_num_) = ray_  ;
            ray_stack_d_(row_num_,col_num_,:) = ray_.d  ;
            ray_stack_P0_(row_num_,col_num_,:) = ray_.P0  ;             
            % {
            if mod(col_num_-1,10)==0 && mod(row_num_-1,10) ==0
                plot3_rows(  ... %  plot the rays through the pixel centres - every 10th*10th to keep it reasonable 
                    [ ...
                    squeeze(  ray_stack_P0_(row_num_,col_num_,:))' ; 
                    ( 5.0*squeeze(  ray_stack_d_(row_num_,col_num_,:))+squeeze( ray_stack_P0_(row_num_,col_num_,:)) )'   ...
                    ]' , 'm' )  ;
            end
            % }
            % calculate intercept with bottom of robot / floor 
            [ z_eq_0_intercept( row_num_ , col_num_ , : ) , ...
              scaling_factor_for_vector_along_line_to_intercept( row_num_ , col_num_ ) ]  ...
                = geom__z_plane_intercept(  ...
                    squeeze(ray_stack_P0_(row_num_,col_num_,:)) , ...
                    squeeze(ray_stack_d_(row_num_,col_num_,:))  , 0 )  ;
%             plot3_rows( z_eq_0_intercept , 'bs' )  ;
            % calculate intercept with top of robot
            [ z_eq_2_intercept( row_num_ , col_num_ , : ) , ...
              scaling_factor_for_vector_along_line_to_intercept_2( row_num_ , col_num_ ) ]  ...
                = geom__z_plane_intercept(  ...
                    squeeze(ray_stack_P0_(row_num_,col_num_,:)) , ...
                    squeeze(ray_stack_d_(row_num_,col_num_,:))  , 2 )  ;
%             plot3_rows( z_eq_2_intercept , 'rs' )  ;
        end
    end
    %  figure; histogram2( ray_stack_P0_(:,:,1) , ray_stack_P0_(:,:,2) ) ;    figure; histogram2( ray_stack_P0_(:,:,3) , ray_stack_P0_(:,:,2) ) ;
        
    xlim( [-1,12] )  ;  ylim( [-3,10] )  ;  zlim( [0,3] )  ;

%     graph_cameraposition = get(gca, 'CameraPosition')  ;
%     graph_cameratarget = get(gca, 'CameraTarget')  ;
%         %  set( gca , 'CameraPosition' , graph_cameraposition+[10 15 15] )
%         %  set(gca, 'CameraTarget' , [ 3.0 2.0 1.5] )
%         %  set(gca, 'CameraPosition', [100 5000 2000]);
        
    % draw the z=0 intercept points at 2m height    
%     plot3_rows(  reshape(z_eq_0_intercept, [121,3] )'+repmat([0;0;0],1,121)     , 'mx' )
%     plot3_rows(  reshape(z_eq_2_intercept, [121,3] )'+repmat([0;0;0],1,121)     , 'rx' )
    
    
    %{
    Now have the intercepts with top and bottom of robot volume (ignoring intermmediate obstacles for now) , 
        project those to the planning grid/occupancy grid/map
        identify the set of planning grid nodes that we can see
        score them by information utility
        plan paths through them 
        ALT 
            ask the robot to do its thing again, but go 1m to its left 
    Information utility score ~ approx as difference from existing pixels + difference from existing 3D    
    %}
    
    %-- as points on the ground (z=0) so that I can see them 
   
    % NOW: cull these by (1) image formation + algorithm maximum resolution  (2) obstacles on floor plan  (3) non-free space
    ground_points = reshape(z_eq_0_intercept, [  size(z_eq_0_intercept,1) * size(z_eq_0_intercept,2)  ,3] )'  ;
    ground_points = [ ground_points  reshape(z_eq_2_intercept, [ size(z_eq_0_intercept,1) * size(z_eq_0_intercept,2) ,3] )' ]  ;
    plot3(ground_points(1,:),ground_points(2,:), repmat(  -0.01  , 1, size(ground_points,2) ), 'ro')    
    
    %-- convert from world points to grid axis - nothing to do as we're using an axis-aligned grid
    
    %-- occupancy grid cells
    % determine which grid point they're in
    grid_scale = 0.1 ;  %0.1m per grid cell
    grid_cell_indices_for_intercepts = ceil(ground_points./grid_scale)  ;
    %  figure; histogram2(grid_cell_indices_for_intercepts(1,:),grid_cell_indices_for_intercepts(2,:))  % eyeball 
    
    %-- objective function --
    %   objective function - information utility of the points to be passed through / of the trajectory to 
    %       information utility of the observations 
    %           --> of the trajectory poses & velocity 
    %               --> of the points/poses passed through
    %  distance is unbounded - how to deal with ?
    %  image is bounded 
    %  distance from the existing observations in image space 
    %
    % for each potential observation, have the pixel coordinate from the ray, and the 3D coordinate
    % from the ray intersect the plane at height of the feature 
    % for each ray, for each feature, determine whether the robot pose is in free space - calibrating one camera, so limit to what that camera can observe
    %   ? Class of RayFeatureGrid .p0 .dir .feature_grid_cells ?   -  data structure to store this genericly
    
    % NOW: cull these by (1) image formation + algorithm maximum resolution  (2) obstacles on floor plan  (3) non-free space
    feature_0 = [0;0;0]  ;
    feature_2 = [0;0;2]  ;
    feature_0_3Ddist_per_px = zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;
    feature_2_3Ddist_per_px = zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;
    distance_from_other_pixels = zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;
   uncertainty_from_imgForm_alg_f0 =  zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;  % multiplier 
   uncertainty_from_imgForm_alg_f2 =  zeros( uu_num_rows_+1 , vv_num_cols_+1 )  ;  % multiplier 
   % ??  uncertainty_normalised =  ?? do I want to normalise this : it's not a trade-off, it's a limit ?? 
   % NOW: cull these by (1) image formation + algorithm maximum resolution  (2) obstacles on floor plan  (3) non-free space
   % !! free_space =   ; % probability: 1|0  = free|obstacle  : multiplier 
    row_num_ = 0 ; col_num_ = 0 ;
    for uu_ = 0 : uu_step_ : uu_num_rows_*uu_step_
        row_num_ = row_num_ + 1 ; 
        col_num_ = 0  ;
        for vv_ = 0 : vv_step_ : vv_num_cols_*vv_step_ 
            col_num_ = col_num_ + 1 ;       
            pixel_coordinate = [ uu_ ; vv_ ]  ;
             diffs = points_2D_preconditioned - repmat( pixel_coordinate,1, size(points_2D_preconditioned,2))  ;
             distance_from_other_pixels(row_num_,col_num_) = min( norm_2(diffs,1) ); 
             
             points_3D_preconditioned_z0 = points_3D_preconditioned ( :,(...
                 points_3D_preconditioned(3,:) >=  feature_2_pose_SE3(3,4)-0.1 ...
                    &  points_3D_preconditioned(3,:) <=  feature_2_pose_SE3(3,4)+0.1)   );
             points_3D_preconditioned_z2 = points_3D_preconditioned (:, ( ...
                 points_3D_preconditioned(3,:) >=  feature_1_pose_SE3(3,4)-0.1 ...
                    &  points_3D_preconditioned(3,:) <=  feature_1_pose_SE3(3,4)+0.1)   );
                
             diffs_feature_0_3D = points_3D_preconditioned_z0 ...
                 - repmat(squeeze(z_eq_0_intercept(row_num_,col_num_,:)),1,size(points_3D_preconditioned_z0,2))  ;
             feature_0_3Ddist_per_px(row_num_,col_num_) = min(norm_2(diffs_feature_0_3D,1))  ;
             
             diffs_feature_2_3D = points_3D_preconditioned_z2 ...
                 - repmat(squeeze(z_eq_2_intercept(row_num_,col_num_,:)),1,size(points_3D_preconditioned_z2,2))  ;
             feature_2_3Ddist_per_px(row_num_,col_num_) = min(norm_2(diffs_feature_2_3D,1))  ;      
             
             extrinsics_approximate_uncertainty_distance_func(norm_2(squeeze(z_eq_2_intercept(row_num_,col_num_,:)) - camera.T(1:3,4),1));             
                
             uncertainty_from_imgForm_alg_f0(row_num_,col_num_) = ...
                 extrinsics_approximate_uncertainty_distance_func( ...
                    norm_2(squeeze(z_eq_0_intercept(row_num_,col_num_,:)) - best_pose(1:3,4),1));
                
             uncertainty_from_imgForm_alg_f2(row_num_,col_num_) = ...
                 extrinsics_approximate_uncertainty_distance_func( ...
                    norm_2(squeeze(z_eq_2_intercept(row_num_,col_num_,:)) - best_pose(1:3,4),1));
        end
    end        
   pgood_detection_f0 = 1 - uncertainty_from_imgForm_alg_f0  ;  % multiplier  in range 1:0 = accurate detection:inaccurate detection 
   pgood_detection_f2 = 1 - uncertainty_from_imgForm_alg_f2  ;
    % per ray --> WRONG for this, as SHOULD have value per 3D-2D observation, and then SUM per grid cell / pose to plan to
%     figure; surf( ... % balance equally between the 3D and 2D gain - is this a good thing w.r.t. EPnP ?
%         (feature_0_distance_3D./max(max(feature_0_distance_3D)) ...
%         + feature_2_distance_3D./max(max(feature_2_distance_3D)) ...
%         )./2 ...
%         + distance_from_other_pixels./max(max(distance_from_other_pixels)))  ;    
    figure_named('1*feature_0_distance_3D + 1*distance_from_other_pixels'); surf( ... % balance equally between the 3D and 2D gain - is this a good thing w.r.t. EPnP ?
        feature_0_3Ddist_per_px./max(max(feature_0_3Ddist_per_px)) ...
        + distance_from_other_pixels./max(max(distance_from_other_pixels)))  ;    
    figure_named('1*feature_2_distance_3D + 1*distance_from_other_pixels'); surf( ... % balance equally between the 3D and 2D gain - is this a good thing w.r.t. EPnP ?
        feature_2_3Ddist_per_px./max(max(feature_2_3Ddist_per_px)) ...
        + distance_from_other_pixels./max(max(distance_from_other_pixels)))  ;
    % this shows that the nearby cells will dominate in a summation over observations per ray per cell because they get more rays passing through the grid cells
    % on the plane
    %  --> need to find the biggest-benefit per cell : can only observe once per frame per ray per pose 
    %       --> OR find the biggest-benefit per pixel : 
    figure_named('histogram2(z_eq_0_intercept'); histogram2(z_eq_0_intercept(:,:,1)./grid_scale,z_eq_0_intercept(:,:,2)./grid_scale)
    
   % NOW: cull these by (1) image formation + algorithm maximum resolution  (2) obstacles on floor plan  (3) non-free space

   
    feature_0_3Ddist_per_px_normalised = feature_0_3Ddist_per_px./max(max(feature_0_3Ddist_per_px))  ;
    feature_2_3Ddist_per_px_normalised = feature_2_3Ddist_per_px./max(max(feature_2_3Ddist_per_px))  ;
    distance_from_other_pixels_normalised = distance_from_other_pixels./max(max(distance_from_other_pixels))  ;
    %  figure_named('feature_0_3Ddist_per_px_normalised'),surf(feature_0_3Ddist_per_px_normalised);
    %  figure_named('feature_2_3Ddist_per_px_normalised'),surf(feature_2_3Ddist_per_px_normalised);
    figure_named('distance_from_other_pixels_normalised '); surf(distance_from_other_pixels_normalised);
    
    
    best_feature_per_ray = zeros(size(feature_0_3Ddist_per_px )) ;
    best_feature_per_ray(feature_0_3Ddist_per_px > feature_2_3Ddist_per_px) = 0  ;    % ??
    best_feature_per_ray(feature_0_3Ddist_per_px <= feature_2_3Ddist_per_px) = 2  ;  % ??
    best_feature_per_ray(feature_0_3Ddist_per_px_normalised.*pgood_detection_f0 > feature_2_3Ddist_per_px_normalised.*pgood_detection_f2) = 0  ;  % ??
    best_feature_per_ray(feature_0_3Ddist_per_px_normalised.*pgood_detection_f0 <= feature_2_3Ddist_per_px_normalised.*pgood_detection_f2) = 2  ;  % ??
    figure_named('surf(best_feature_per_ray)'); surf(best_feature_per_ray)
    
    best_feature_per_ray(feature_0_3Ddist_per_px.*pgood_detection_f0 <= feature_2_3Ddist_per_px.*pgood_detection_f2) = 2  ;  % ??
    best_feature_per_ray(feature_0_3Ddist_per_px.*pgood_detection_f0 > feature_2_3Ddist_per_px.*pgood_detection_f2) = 0  ;  % ??
    figure_named('surf(best_feature_per_ray)'); surf(best_feature_per_ray)
    
    best_3Ddist_per_ray = zeros(size(feature_0_3Ddist_per_px )) ;
    best_3Ddist_per_ray(best_feature_per_ray==0) =  feature_0_3Ddist_per_px_normalised(best_feature_per_ray==0).*pgood_detection_f0(best_feature_per_ray==0);
    best_3Ddist_per_ray(best_feature_per_ray==2) =  feature_2_3Ddist_per_px_normalised(best_feature_per_ray==2).*pgood_detection_f2(best_feature_per_ray==2);
    
    figure_named('surf(best_3Ddist_per_ray)'); surf(best_3Ddist_per_ray)
    figure_named('feature_0_3Ddist_per_px_normalised.*pgood_detection_f0');  surf(feature_0_3Ddist_per_px_normalised.*pgood_detection_f0)
    figure_named('feature_2_3Ddist_per_px_normalised.*pgood_detection_f2');  surf(feature_2_3Ddist_per_px_normalised.*pgood_detection_f2)
        hold on ;  surf(feature_0_3Ddist_per_px_normalised.*pgood_detection_f0) ;
    
    payoff_per_ray = 3.0*best_3Ddist_per_ray + distance_from_other_pixels_normalised ;  % What is the balance here? 
    figure_named('payoff_per_ray');  surf(payoff_per_ray)
    
    %-- the grid location of the best feature in each ray/pixel 
    robot_posn_per_ray = zeros( [ size(feature_0_3Ddist_per_px ) , 3 ]) ;
    [xxx_,yyy_]=find(best_feature_per_ray==2)  ;
    robot_posn_per_ray( xxx_,yyy_ , :   ) = z_eq_2_intercept( xxx_,yyy_ , : )  ;
    [xxx_,yyy_]=find(best_feature_per_ray==0)  ;
    robot_posn_per_ray( xxx_,yyy_ , :   ) = z_eq_0_intercept( xxx_,yyy_ , : )  ;
    %  figure; surf(robot_posn_per_ray(:,:,3))
    
    robot_posn_per_ray_xy = robot_posn_per_ray(:,:,1:2)  ; 
    figure_named('histogram2(robot_posn_per_ray_xy(:,:,1)');  histogram2(robot_posn_per_ray_xy(:,:,1),robot_posn_per_ray_xy(:,:,2))
    
    %  per ray/pixel find the grid cell it adds to, and cache the best payoff for that grid cell 
    map_limits_cells=[-200 -200 200 200]   ;
    offset_x = 200 ;
    offset_y = 200 ;
    grid_cells_best_payoffs = zeros(abs(map_limits_cells(1))+map_limits_cells(3),abs(map_limits_cells(2))+map_limits_cells(4))  ;
    grid_cells_best_payoffs_rays = zeros(abs(map_limits_cells(1))+map_limits_cells(3),abs(map_limits_cells(2))+map_limits_cells(4), 2, 'uint32')  ;
    num_outside = 0;
    row_num_ = 0 ; col_num_ = 0 ;
    for uu_ = 0 : uu_step_ : uu_num_rows_*uu_step_
        row_num_ = row_num_ + 1 ; 
        col_num_ = 0  ;
        for vv_ = 0 : vv_step_ : vv_num_cols_*vv_step_ 
            col_num_ = col_num_ + 1 ;       
            pixel_coordinate = [ uu_ ; vv_ ]  ;
            grid_cell_xy = ceil( squeeze(robot_posn_per_ray_xy(row_num_,col_num_  , :))./grid_scale )  ;
            if grid_cell_xy(1)+offset_x > map_limits_cells(3)+offset_x ; display('over x'); display(grid_cell_xy(1)+offset_x);  num_outside=num_outside+1; 
            elseif grid_cell_xy(2)+offset_y > map_limits_cells(4)+offset_y ; display('over y'); display(grid_cell_xy(2)+offset_y);   num_outside=num_outside+1; 
            elseif grid_cell_xy(1)+offset_x < map_limits_cells(1)+offset_x ; display('under x'); display(grid_cell_xy(1)+offset_x);   num_outside=num_outside+1; 
            elseif grid_cell_xy(2)+offset_y < map_limits_cells(2)+offset_y ; display('under y'); display(grid_cell_xy(2)+offset_y);  num_outside=num_outside+1; 
            else
                if grid_cells_best_payoffs( grid_cell_xy(1)+offset_x,grid_cell_xy(2)+offset_y) < payoff_per_ray(row_num_,col_num_) ...
                    || ...
                	grid_cells_best_payoffs( grid_cell_xy(1)+offset_x,grid_cell_xy(2)+offset_y) == payoff_per_ray(row_num_,col_num_) && rand>=0.5
                    grid_cells_best_payoffs( grid_cell_xy(1)+offset_x,grid_cell_xy(2)+offset_y) = payoff_per_ray(row_num_,col_num_);
                    grid_cells_best_payoffs_rays( grid_cell_xy(1)+offset_x,grid_cell_xy(2)+offset_y,:) = [row_num_,col_num_]  ;
                end
            end
        end
    end
    %    figure; surf(grid_cells_best_payoffs)
    %   figure; histogram(grid_cells_best_payoffs,100);  set(gca,'YScale','log')
    max(max(grid_cells_best_payoffs))
    size(sort(grid_cells_best_payoffs(:)))
    
    [sorted_payoffs,sorted_payoffs_indices]=sort(grid_cells_best_payoffs(:),'descend')  ;
    [sorted_payoffs_rows_,sorted_payoffs_cols_]=ind2sub(size(grid_cells_best_payoffs),sorted_payoffs_indices)  ;
    
    threshold_utility = 1.2;  %  0.8 ;
    suppression_radius_m = 1.2 ;
    suppression_radius = ceil(suppression_radius_m/grid_scale)  ;
    suppressed = zeros(abs(map_limits_cells(1))+map_limits_cells(3),abs(map_limits_cells(2))+map_limits_cells(4))  ;
    used_cells = zeros(abs(map_limits_cells(1))+map_limits_cells(3),abs(map_limits_cells(2))+map_limits_cells(4))  ;
    for ii_=1:size(sorted_payoffs)
        this_r = sorted_payoffs_rows_(ii_);
        this_c = sorted_payoffs_cols_(ii_)  ;
        if ~suppressed(this_r,this_c)
            neighbours=[ this_r-suppression_radius , this_r+suppression_radius...
                this_c-suppression_radius , this_c+suppression_radius  ] ;
            if neighbours(1)<1; neighbours(1)=1; end
            if neighbours(2)>size(grid_cells_best_payoffs,1); neighbours(2)=size(grid_cells_best_payoffs,1); end;
            if neighbours(3)<1;neighbours(3)=1; end;
            if neighbours(4)>size(grid_cells_best_payoffs,2); neighbours(4)=size(grid_cells_best_payoffs,2); end;
            suppressed(neighbours(1):neighbours(2),neighbours(3):neighbours(4)) = 1  ;
            if grid_cells_best_payoffs(this_r,this_c) > threshold_utility
                used_cells(this_r,this_c) = 1  ;
            end
            % display(' !  CLICK TO CONTINUE  ! ')
            %pause
        else
            % display('skipped')
        end    
    end
    figure_named('used_cells');surf(used_cells);  
        hold on; plot3_rows( best_pose(1:3,4).*[1/grid_scale;1/grid_scale;1] + [offset_x;offset_y;0],'rx' )
        hold on; plot3_rows( best_pose(1:3,4).*[1/grid_scale;1/grid_scale;0] + [offset_x;offset_y;0],'ro' )
    figure_named('histogram(used_cells'); histogram(used_cells,100); set(gca, 'YScale', 'log')
    figure_named('used_cells.*grid_cells_best_payoffs');surf(used_cells.*grid_cells_best_payoffs);  
    figure_named('histogram(used_cells.*grid_cells_best_payoffs'); histogram(used_cells(used_cells>0).*grid_cells_best_payoffs(used_cells>0),100); xlabel('used\_cells.*grid\_cells\_best\_payoffs')
    
    vector_scaling_for_figure=     [1/grid_scale;1/grid_scale;1.0]  ;
    figure_named('surf( used_cells.*grid_cells_best_payoffs )');  surf( used_cells.*grid_cells_best_payoffs ) ;  hold on; plot3( 0+offset_x , 0+offset_y , 0, 'bo' )
        plot3_rows(points_3D_f1.*repmat(vector_scaling_for_figure,1,size(points_3D_f1,2))+repmat([offset_x;offset_y;0],1,size(points_3D_f1,2)),'rx')
        plot3_rows(points_3D_f2.*repmat(vector_scaling_for_figure,1,size(points_3D_f2,2))+repmat([offset_x;offset_y;0],1,size(points_3D_f2,2)),'mx')
        plot3_rows(points_3D_f3.*repmat(vector_scaling_for_figure,1,size(points_3D_f3,2))+repmat([offset_x;offset_y;0],1,size(points_3D_f3,2)),'gx')
        axes_scale = 1;   draw_axes_direct_c( models_extrinsic_estimate_as_local_to_world(1:3,1:3,best_model_id).*(1/grid_scale) , models_extrinsic_estimate_as_local_to_world(1:3,4,best_model_id) , '' ,  axes_scale , 'r'    )  ;                    
        zlim([-0.1,5])               
    ray_length=1.0 ; %1/grid_scale;
    ray_scaling_for_figure=     [ray_length/grid_scale;ray_length/grid_scale;1.0]  ;
    for row_num_ = 1:10:size(ray_stack_P0_,1)
        for col_num_ = 1:10:size(ray_stack_P0_,2)
                    plot3_rows(  ... %  plot the rays through the pixel centres 
                [ ...
                (squeeze(  ray_stack_P0_(row_num_,col_num_,:)).*[1/grid_scale;1/grid_scale;1.0])'+[offset_x,offset_y,0] ; 
                (squeeze(z_eq_0_intercept(row_num_,col_num_,:)).*ray_scaling_for_figure+[offset_x;offset_y;0])'  ...
                %(   (  squeeze(  ray_stack_d_(row_num_,col_num_,:)).*ray_scaling_for_figure+squeeze( ray_stack_P0_(row_num_,col_num_,:)).*[1/grid_scale;1/grid_scale;1.0] )+[offset_x;offset_y;0] )'   ...
                ]' , 'r' )  ;
        end
    end
    figure_named('histogram( used_cells.*grid_cells_best_payoffs )'); histogram( used_cells.*grid_cells_best_payoffs );  set(gca, 'YScale', 'log')
    sum(sum(used_cells))
    figure_named('surf(grid_cells_best_payoffs)'); handle_ = surf(grid_cells_best_payoffs) ;  hold on; plot3( 0+offset_x , 0+offset_y , 0, 'bo' ); xlabel('x'); ylabel('y')
    
    plot3_rows( [offset_x ; offset_y ; 0]-best_pose(1:3,4).*[1/grid_scale;1/grid_scale;0]   , 'mx' );    
    %  !!!  COORDINATE  DIRECTIONS  ARE  SWAPPED  !!! : work out how to get SURF to use the other coordinate direction or some such 
    
    %%
    %{ 
    We have the grid cells to use, now determine some trajectories, the utility of those to the camera, and the approx length/cost to the robot. 
    (1) directional - want same traversal to deal with latency 
    (2) assume same path 
    (3) KISS, JFDI 
    %}
    % Put the cells in order of payoff
    
    % Find paths through them
    %     if n < 6    % exhaustive - up to 120 - but for ALL 
    %         %  perms
    %     else  
    %         for ii_ = 1:120 % stochastic - 120 - but for ALL 
    %             % randperm
    %         end
    %     end
    ti=find(used_cells>0)  ;
    [tr,tc]=find(used_cells>0)  ;
    grid_cells_best_payoffs(ti)  ;
    num_paths = 100 ;
    max_num_waypoints = 4  ;
    waypoints_set = zeros(num_paths, max_num_waypoints)  ;
    for ii_ = 1:num_paths
        ti2 = ti ;
        path_size = 1;
        ti2_order_to_pick =  randperm( size(ti2,1) )  ; 
        waypoints = zeros(1,max_num_waypoints) ;
        waypoints(path_size) = ti2(ti2_order_to_pick(path_size)) ;
        if rand >= 0.5
            path_size = 2;
            waypoints(path_size) = ti2(ti2_order_to_pick(path_size)) ;
            if rand >= 0.5 
                path_size = 3;                                
                waypoints(path_size) = ti2(ti2_order_to_pick(path_size)) ;
                if rand >= 0.5 
                    path_size = 4;                            
                    waypoints(path_size) = ti2(ti2_order_to_pick(path_size)) ;
                end
            end
        end
        waypoints_set(ii_,:) = waypoints;
    end
    waypoints_set = unique(waypoints_set,'rows') ;
    
    grid_scale_inv = 1/grid_scale  ;
    [waypoints_set_grid_cells_r waypoints_set_grid_cells_c] = ind2sub(size(grid_cells_best_payoffs), waypoints_set)  ;
    waypoints_set_grid_cells_c(waypoints_set_grid_cells_r == 0) = 0
    waypoints_set_grid_cells_y = waypoints_set_grid_cells_r./grid_scale_inv
    waypoints_set_grid_cells_x = waypoints_set_grid_cells_c./grid_scale_inv
    waypoints_set_grid_cells_xy=cat(3,waypoints_set_grid_cells_x,waypoints_set_grid_cells_y)
    
    waypoints_ = waypoints_set_grid_cells_y(64,waypoints_set_grid_cells_y(64,:)~=0)
    robot_entry = 0
    robot_exit = 10 
    robot_motion_model_vel_limits = [ 1 1 ]
    robot_motion_model_acc_limits = [ 0.5 0.5 ]
    start_point = robot_entry 
    via_points = [ waypoints_ robot_exit ]
    num_via_points = size(via_points)
    %  TRAJ = MSTRAJ(P, QDMAX, TSEG, Q0, DT, TACC, OPTIONS)
    mstraj( via_points , robot_motion_model_acc_limits , [] , start_point , 0.1 , robot_motion_model_acc_limits )

e.g.
    traj_ = mstraj( [ 1 1 ; 2 1 ; 3 0  ], [1.2 1.2]  , [] , [ 0 0 ] , 0.1 , 1  )
    size(traj_)
    traj_ = mstraj( [ 1 1 ; 2 1 ; 3 0  ], [1.2 1.2]  , [] , [ 0 0 ] , 0.2 , 1  )
    size(traj_)
    traj_ = mstraj( [ 1 1 ; 2 1 ; 3 0  ], [0.2 0.2]  , [] , [ 0 0 ] , 0.2 , 1  )
    
    
    %% -----------------------------------------------------------------
    %%
%%
    %  see  camera_extrinsics__test_optical_axis_intercept_plane.m  -  section  "use the refactored functions to flip the camera"
    display( 'do PCA' )    
    
    camera_old = camera ;    
    
    %   see   /mnt/nixbig/ownCloud/project_code/camera_extrinsics__test_optical_axis_intercept_plane.m
    
        %     [coeff, score, latent, tsquared, explained, mu]  =  pca( [feature_1_positions feature_2_positions feature_3_positions]' ) ;
    [coeff, score, latent, tsquared, explained, mu]  =  pca( [ points_3D_f1_latency points_3D_f3_latency points_3D_f3_latency ]' ) ;
        hold on;  plot3_rows( mu' , 'mo' ) ;  plot3_rows( [ mu'-(coeff(:,1)*10) mu'+(coeff(:,1)*10) ] , 'm') ;    plot3_rows( [ mu'-(coeff(:,2)*10) mu'+(coeff(:,2)*10) ] , 'c') ;    plot3_rows( [ mu'-(coeff(:,3)*10) mu'+(coeff(:,3)*10) ] , 'k')
        
    pca_second_axis = [ mu'-(coeff(:,2)*10) mu'+(coeff(:,2)*10) ] ;
    pca_second_axis = [ pca_second_axis(1:2,1:2) ;  0 0]  ;
    plot3_rows(pca_second_axis , 'ks')
    plot3_rows(pca_second_axis(:,1) , 'rd')
    plot3_rows(pca_second_axis(:,2) , 'bd')  %  THIS direction 
    
    pca_first_axis = [ mu'-(coeff(:,1)*10) mu'+(coeff(:,1)*10) ] ;
    pca_first_axis = [ pca_first_axis(1:2,1:2) ;  0 0]  ;
    plot3_rows(pca_first_axis , 'ks')
    
    % want to generate a set of points :  
        %   Option 1. A path parallel to the one already observed, but shifted in the direction of pca_second_axis
        %       Then check for visibility in the current pose estimate - Then WITH uncertainty.
        %       ( Then check for occlusion by floorplan. )
        %       NOTE:  _path_ _trajectory_, not just points  -->
        %           --> control the latency direction, control the velocity so can get expected number of observations
        %   Option 1.2. As Option 1 - a path parallel to the one already observed, but shifted in the direction of pca_second_axis 
        %       - BUT give as envelopes that the robot can pass through rather than specific points/trajectories 
        %   Option 2. Create a grid of points parallel to pca_first_axis , but shifted incrementally in the direction of pca_second_axis
        %       AND culled by edges of Field of View 
        %       ( Then culled for occlusion by floorplan. )
        %       THEN joined into short trajectories :  send the trajectories to the robot :  ' trajectories'  are poses and max,preferred velocities  -  max="this or
        %           slower" min="best vel for me (cam); you could go slower than this if it suits you"
        %
        %   Probably instantiate pose demands in A* as bracketing end point with lead-in obstacles parallel to the goal
        %       OR by setting poses along Beziers as with mtraj() function. 
        %   Plan the global path as a series of A* plans from each pose to the next
        
    % Option 2. Part 1. Generate a grid of points         
    offset_vec = pca_second_axis(:,2)  ;   %  THIS direction 
    offset_vec = offset_vec ./ norm_2(offset_vec,1)  ;
    p_num_new_path_points = ceil(sqrt( (pca_first_axis(1,1) - pca_first_axis(1,2))^2 + (pca_first_axis(2,1) - pca_first_axis(2,2))^2 ))  ;
    new_path_points = [  ...
        linspace(pca_first_axis(1,1) , pca_first_axis(1,2) , p_num_new_path_points )  ;
        linspace(pca_first_axis(2,1) , pca_first_axis(2,2) , p_num_new_path_points )  ;
        zeros(1,p_num_new_path_points)  ]  ;
    
    num_its = 11;
    new_path_points_0 = new_path_points  ;
    new_path_points_keep = []  ;
    new_path_points_keep_count = zeros(1,num_its) ; 
    
    for ii_ = 1: num_its
        new_path_points_0 = new_path_points_0+repmat(offset_vec,1,size(new_path_points_0,2));

        % Part 2. Cull if outside field of view
        new_path_points_uv = camera.project(   new_path_points_0  )  ;
        % figure;  hold on; grid on; axis equal; plot2_rows(  camera.project(    new_path_points  )  ) ;  plot2_rows( [ 0 0 ; 0 1024 ; 1024 1024 ; 1024 0 ]', 'bo' )  % check: eyeball
        new_path_points_indices_1  =  ( new_path_points_uv(1,:)>=0 & new_path_points_uv(1,:) <= 1024 & new_path_points_uv(2,:)>=0 & new_path_points_uv(2,:) <= 1024 )  ;
        new_path_points_uv_1 = new_path_points_uv( : , new_path_points_indices_1 )  ;
        new_path_points_1 = new_path_points_0( : , new_path_points_indices_1 )  ;
        new_path_points_1 = [ new_path_points_1 zeros( 3, 100-size(new_path_points_1,2)) ]  ;
        %  2D figure  -  plot2_rows(  new_path_points_uv_1   , 'gx' )    %  eyeball
        %  3D figure  -  plot3_rows(  new_path_points_1   , 'gx' )    %  eyeball
        plot3_rows(  new_path_points_1   , 'gx' )    %  eyeball
        plot3_rows(  new_path_points_1   , 'go' )    %  eyeball
        if size(new_path_points_uv_1,2)>0
            new_path_points_keep = cat( 3, new_path_points_keep , new_path_points_1 )  ;            
            new_path_points_keep_count(1,ii_) =  size(new_path_points_uv_1,2);
        end
%         drawnow
%         pause
    end
    
    
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
    