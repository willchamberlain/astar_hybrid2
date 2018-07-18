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
%%
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
    for ii_ii_ = 1: num_camera_posn
        target_point3D = points_3D_f1(:,indices_(ii_ii_) )  ;
        % Orient camera orthogonal to the x axis, and zero roll
        %  see camera_extrinsics__place_camera_2.m
        cam_pose_xyz = target_point3D + [ 0.0 -3.0 2.0 ]'  ;  % 3m away, 2m up
        cam_pose_xyz = target_point3D + [ -1.0 -2.2 2.0 ]'  ;  % 3m away, 2m up
        cam_pose_xyz = target_point3D + [ -2.0 -2.2 2.0 ]'  ;  % 3m away, 2m up
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
           [ models , models_max_diff_SE3_element , models_extrinsic_estimate_as_local_to_world ] = ...  % , models_best_solution , models_solution_set ] = ...           
        camera_extrinsics__iterate_epnp  ( ...
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
            if sum(reprojection_Euclidean_pts(:,:,ii_)<=mean_reprojection_inliers_threshold) > 0
                mean_reprojection_inliers_exclude_above_threshold(ii_)=mean(reprojection_Euclidean_pts(:,reprojection_Euclidean_pts(:,:,ii_)<=mean_reprojection_inliers_threshold, ii_)) ;
            else
                mean_reprojection_inliers_exclude_above_threshold(ii_)=nan;
            end
        end
        
        %  draw black axes for each of the estimates below a threshold
        %         models_with_mean_reprojection_error_below_threshold__logical = (reprojection_Euclidean_total/num_points < quantile([reprojection_Euclidean_total/num_points]',0.10))  ; 
        %         mwmrebt__logical = models_with_mean_reprojection_error_below_threshold__logical   ;
        %         figure; histogram(models_with_mean_reprojection_error_below_threshold__logical)  ;
        %         mwmrebt__pose_estimate = models_extrinsic_estimate_as_local_to_world(:,:,models_with_mean_reprojection_error_below_threshold__logical)  ;
        % %             figure;  
        %             for ii_ = 1:size(mwmrebt__pose_estimate,3)
        %                 draw_axes_direct_c( mwmrebt__pose_estimate(1:3,1:3,ii_) , mwmrebt__pose_estimate(1:3,4,ii_) , '' ,  2 , 'k'    )  ;
        %             end
        %  draw black axes for each of the estimates below a threshold
%         figure; grid on; axis equal;
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
        
        
        %         mean_reprojection_error_below_threshold__dist_HalfNormal = fitdist([reprojection_Euclidean_total/num_points]','HalfNormal')  ;
        %         mean_reprojection_error_below_threshold__dist_Exponential = fitdist([reprojection_Euclidean_total/num_points]','Exponential')  ;        
 

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
% %     fighandle_posn_error = ... 
% %         figure('Name',strcat(exp_num,' : ','position error'));  fighandle_posn_error.Name='Estimated position errors'  ; grid on; xlabel('estimatenumber'); ylabel('euclidean distance from true camera position');
    posn_euclidean_dist_error = norm_2( estimated_position_diffs, 1)  ;
% %     hold on; plot(posn_euclidean_dist_error,'bx')  ;
% %     plot( estimated_position_diffs(1,:) , 'rs' )  ; plot( estimated_position_diffs(2,:) , 'gs' )  ;plot( estimated_position_diffs(3,:) , 'bs' )  ;
% %     figure(fighandle_posn_error); legend('x','y','z')
% %     
% %     fighandle_posn_error_log = figure('Name',strcat(exp_num,' : ','position error'));   semilogy( estimated_position_diffs(1,:) , 'rs' )  ; hold on; semilogy( estimated_position_diffs(2,:) , 'gs' )  ;semilogy( estimated_position_diffs(3,:) , 'bs' )  ;
% %     grid on; xlabel('estimatenumber'); ylabel('euclidean distance from true camera position');     
% %     figure(fighandle_posn_error_log); legend('x','y','z')
    
        %      fig_now_=gcf
        %      fig_now_.Position=[808 460 907 414]
        
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
%{
if false && size(estimated_position_diffs_under_reproj_5,2) > 5
     fig_now_ = figure('Name',strcat(exp_num,' : #',sprintf('%d',ii_ii_), sprintf('x=%3.3f, y=%3.3f, z=%3.3f', camera.T(1,4), camera.T(2,4), camera.T(3,4)) , ': error distribution under reprojection: ' ));  
     set(fig_now_, 'Position',  [155   165   655  785] ) ;
     ylim_ = [0 350]  ;
     ylim_ = [0 min(p_num_RANSAC_iterations, 100)]  ;
     subplot(3,3,1); histogram(estimated_position_diffs_under_reproj_5(1,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]);  title('Position error for reprojection error <= 5px'); xlabel(' x(m) '); ylim(ylim_);
    %      subplot(3,3,1); histogram(estimated_position_diffs(1,:)); xlim('auto')
     subplot(3,3,2); histogram(estimated_position_diffs_under_reproj_5(2,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' y(m) '); ylim(ylim_);  
     subplot(3,3,3); histogram(estimated_position_diffs_under_reproj_5(3,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' z(m) '); ylim(ylim_);
     A = histcounts(estimated_position_diffs_under_reproj_5(1,:), 100);
     ylim_ = [0 min(min(p_num_RANSAC_iterations, 100), max(max(A),10))] 
         subplot(3,3,4); histogram(estimated_position_diffs_under_reproj_5(1,:), 100); xlabel(' x(m) '); ylim(ylim_);
         subplot(3,3,5); histogram(estimated_position_diffs_under_reproj_5(2,:), 100); xlabel(' y(m) '); ylim(ylim_);  
         subplot(3,3,6); histogram(estimated_position_diffs_under_reproj_5(3,:), 100); xlabel(' z(m) '); ylim(ylim_); 
     subplot(3,3,7); histogram(reprojection_Euclidean_mean, 100); title('Mean reprojection error'); xlabel(' pixels '); xlim([0 100]);
end     
if size(estimated_position_diffs_under_dist,2) > 5
     figure('Name',strcat(exp_num,' : #',sprintf('%d',ii_ii_),': error distribution under distance: ', sprintf('x=%3.3f, y=%3.3f, z=%3.3f', camera.T(1,4), camera.T(2,4), camera.T(3,4)) ));  
     ylim_ = [0 350]  ;
     ylim_ = [0 min(p_num_RANSAC_iterations, 100)]  ;
     subplot(3,3,1); histogram(estimated_position_diffs_under_dist(1,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' x(m) '); ylim(ylim_);
     subplot(3,3,2); histogram(estimated_position_diffs_under_dist(2,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' y(m) '); ylim(ylim_);  
     subplot(3,3,3); histogram(estimated_position_diffs_under_dist(3,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' z(m) '); ylim(ylim_);
     A = histcounts(estimated_position_diffs_under_dist(1,:), 100);
     ylim_ = [0 min(min(p_num_RANSAC_iterations, 100), max(max(A),10))]  ;
         subplot(3,3,4); histogram(estimated_position_diffs_under_dist(1,:), 100); xlabel(' x(m) '); ylim(ylim_);
         subplot(3,3,5); histogram(estimated_position_diffs_under_dist(2,:), 100); xlabel(' y(m) '); ylim(ylim_);  
         subplot(3,3,6); histogram(estimated_position_diffs_under_dist(3,:), 100); xlabel(' z(m) '); ylim(ylim_); 
     subplot(3,3,7); histogram(reprojection_Euclidean_mean, 100); title('reprojection\_Euclidean\_mean')
else
     figure('Name',strcat(exp_num,' : #',sprintf('%d',ii_ii_),': error distribution: ', sprintf('x=%3.3f, y=%3.3f, z=%3.3f', camera.T(1,4), camera.T(2,4), camera.T(3,4)) ));  
     subplot(3,3,1); histogram(estimated_position_diffs(1,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' x(m) '); ylim([0 350]);
     subplot(3,3,2); histogram(estimated_position_diffs(2,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' y(m) '); ylim([0 350]);  
     subplot(3,3,3); histogram(estimated_position_diffs(3,:), [-0.5:0.02:0.5]); xlim([-0.5 0.5]); xlabel(' z(m) '); ylim([0 350]); 
     subplot(3,3,4); histogram(estimated_position_diffs(1,:), 100); xlabel(' x(m) '); ylim([0 350]);
     subplot(3,3,5); histogram(estimated_position_diffs(2,:), 100); xlabel(' y(m) '); ylim([0 350]);  
     subplot(3,3,6); histogram(estimated_position_diffs(3,:), 100); xlabel(' z(m) '); ylim([0 350]); 
     subplot(3,3,7); histogram(reprojection_Euclidean_mean, 100); title('reprojection\_Euclidean\_mean')
end       
%}

% %     figure('Name' , strcat( exp_num , ' : ' , 'orientation error distribution' ) )  ;  
% %     histogram(estimated_orientation_diffs, 1000)  ;
% %     
% %     figure('Name' , strcat( exp_num , ' : ' , 'orientation error distribution  _b' ) )  ;  
% %     histogram(estimated_orientation_diffs_2, 1000)  ;
% %     hold on;  plot( [pi pi], [0 40], 'r')  ;  plot( [-pi -pi], [0 40], 'r')  ;  plot( [0 0], [7 40], 'r') ;     %  emphasise the limits 
    
    
    % NEXT HERE %     NEXT
    %     A = [  estimated_orientation_diffs  ,   abs(estimated_orientation_diffs)  ,  2*pi-abs(estimated_orientation_diffs)  ,  min( abs(estimated_orientation_diffs) , 2*pi-abs(estimated_orientation_diffs) ) , estimated_orientation_diffs  ]
    %     A(A(:,1)<pi*-1 , 5)+2*pi
    %     A(A(:,1)<pi*-1 , 5) = A(A(:,1)<pi*-1 , 5)+2*pi
    
    
% %     fighandle_posn_err_3d = ...
% %         figure('Name',strcat(exp_num,' : ','position errors plotted as 3D points: looking for clusters'))  ;  grid on  ;  hold on  ;  
% %     plot3_rows(estimated_position_diffs, 'rx')  ;     
% %     params_as_string = sprintf(' num_points=%d, model_size=%d , num_RANSAC_iterations=%d' , num_points,  model_size , num_RANSAC_iterations ) ;
% %     fighandle_euc_posn_err_hist = ...
% %         figure('Name',  strcat(exp_num,' : ',strcat('Euclidean distance of position errors as hist/density',params_as_string)))  ;  grid on  ;  hold on  ; 
% %     hist(posn_euclidean_dist_error,100)  ;    
% %     fighandle_euc_posn_err_hist2 = ...
% %         figure('Name',  strcat(exp_num,' : ',strcat('Euclidean distance of position errors as hist/density 2',params_as_string)))  ;  grid on  ;  hold on  ; 
% %     histogram(posn_euclidean_dist_error(posn_euclidean_dist_error<1), [ 0:0.01:0.5   0.52:0.02:1.0  ]  )
    
    % works, BUT not necessarily useful: could use difference between rays through a plane 
    %  e.g. the floor plane or a robot feature height plane, normalised by the actual camera ray horizontal distance ,
    %  or the distance along the surface of the 5m radius cylinder wall, or the 3D distance between te 5m projection along the real and estimated optic centres
    
    %{
    sorted_eucidean_distance_error=sort(posn_euclidean_dist_error(posn_euclidean_dist_error<1));
    pd = fitdist(sorted_eucidean_distance_error','half normal')  ;
    %}
    
    
    % { 
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
    % } 
    
    
% %     fighandle_mean_reproj = ...
% %         figure('Name',strcat(exp_num,' : ','mean reprojection_Euclidean_total per RANSAC iteration'));  hold on  ;
% %     semilogy(reprojection_Euclidean_total/num_points, 'rx') ;
% %     semilogy( (reprojection_Euclidean_total.*(reprojection_Euclidean_total>1)/num_points ), 'rs') ;  % higlight the high-magnitude errors 
% %     hold on; xlabel('iteration'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on;    
    
% %     fighandle_mean_reproj_thresh = ...
% %         figure('Name',strcat(exp_num,' : ','mean reprojection_Euclidean_total per RANSAC iteration with mean below 10'));  hold on  ;
% %     semilogy(reprojection_Euclidean_total(reprojection_Euclidean_total/num_points < 10)/num_points, 'rx') ;
% %     % semilogy( (reprojection_Euclidean_total.*(reprojection_Euclidean_total>1)/num_points ), 'rs') ;  % higlight the high-magnitude errors 
% %     hold on; xlabel('iteration'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on;    
    
    
    mean_reprojection_thresholded = reprojection_Euclidean_total(reprojection_Euclidean_total/num_points < 100)/num_points  ;
    mean_reprojection_thresholded = sort(mean_reprojection_thresholded ) ;
% %     fighandle_reproj_sorted = figure('Name',strcat(exp_num,' : ','mean reprojection error, thresholded - sorted')); plot(mean_reprojection_thresholded)
% %     fighandle_reproj_hist1 = figure('Name',strcat(exp_num,' : ','mean reprojection error, thresholded - hist - 100 bins')); hist(mean_reprojection_thresholded,100)   % approximates a half-normal distribution ; very few with almost-zero reprojection, but probably good enough for approx probability distribution and confidence
% %     fighandle_reproj_hist2 = figure('Name',strcat(exp_num,' : ','mean reprojection error, thresholded - hist - 0-100')); histogram(mean_reprojection_thresholded, [ 0:0.1:5 5.2:0.2:10 11:1:20 22:2:100 ]  )
% %     fighandle_reproj_hist3 = figure('Name',strcat(exp_num,' : ','mean reprojection error, thresholded - hist - 0-10')); histogram(mean_reprojection_thresholded, [ 0:0.1:5 5.2:0.2:10 ]  )
% %     
% %     fighandle_reproj_hist4 = figure('Name',strcat(exp_num,' : ','mean reprojection error of inliers, thresholded - hist - 0-10')); histogram(mean_reprojection_inliers_exclude_above_threshold, [ 0:0.1:5 5.2:0.2:10 ]  )
% %     
% %     figure('Name',strcat(exp_num,' : ','pose posn error vs mean reprojection_Euclidean_total'));  
% %     semilogy( posn_euclidean_dist_error , ...
% %         reprojection_Euclidean_total/num_points, 'rx') ;
% %     hold on; xlabel('eucidean_distance_error'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on; 
% %     
% %     figure('Name',strcat(exp_num,' : ','pose posn error vs mean mean reprojection_Euclidean_total'));  
% %     loglog( posn_euclidean_dist_error , ...
% %         reprojection_Euclidean_total/num_points, 'rx') ;
% %     hold on; xlabel('eucidean_distance_error'); ylabel('mean reprojection_Euclidean_total'); hold on; grid on; 
% % 
% %     figure(fig_3d_handle) ; 
    
    %--  SAVE THE RESULTS
    if ~exist('DONT_SAVE_THE_RESULTS','var')
        display( 'SAVE THE RESULTS')
        save_Results_001(exp_num, description ,  camera ,  qb ,  qbd ,  qbdd ,  start_posn ,  via_posns ,  axis_speed_limits ,  time_under_acc ,  time_step ,  latency_s ,  latency_time_steps ,  num_points ,  feature_1_pose_SE3 ,  feature_1_positions ,  points_3D_f1_indices ,  points_3D_f1 ,  points_3D_f1_latency ,  feature_2_pose_SE3 ,  feature_2_positions ,  points_3D_f2_indices ,  points_3D_f2 ,  points_3D_f2_latency ,  p_num_RANSAC_iterations ,  models_extrinsic_estimate_as_local_to_world )  ;
        display( 'SAVED RESULTS')
    else
        display('Did NOT save the results: DONT_SAVE_THE_RESULTS is set.');
    end
    
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
    