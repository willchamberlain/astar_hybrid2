function camera_extrinsics__generate_perfect_data()


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

%%   start: add reprojection error to determine inlier set 
%   see   /mnt/nixbig/ownCloud/project_code/iterate_sample_cam_605_pnp_reproject_2.m
% DONE - consensus size followed by reprojection error
% DONE - added to https://github.com/willchamberlain/project_code 
% NOW - add Gaussian to the 2D points
% LATER - re-estimate from the consensus set
%----  Make some random 3D points : note they're separate by minimum 0.1m  ----%
num_datapoints = 43;  %  5mx3mx1.2m  
type_points_3d='linear'; if strcmpi(type_points_3d, 'random')
    points_volume_dimensions = [6 5 1], points_volume_offsets = [ 0 0 0];
    [ points_3D_random , points_3D_random_hom] = camera_extrinsics__generate_random_3D_data(num_datapoints, points_volume_dimensions, points_volume_offsets);
elseif strcmpi(type_points_3d, 'linear')
    tracks_starts=[  0 0  ;  -1 0 ]'  ; tracks_ends=[  3 6 ; 2 6 ]'  ;  tracks_heights = [ 0.265  0.645 ;  0.265  0.645  ];
    [ points_3D_track_1 , points_3D__track_1_hom] = camera_extrinsics__generate_path_3D_data(num_datapoints, tracks_starts(:,1), tracks_ends(:,1), tracks_heights(:,1))  ;
    [ points_3D_track_2 , points_3D__track_2_hom] = camera_extrinsics__generate_path_3D_data(num_datapoints, tracks_starts(:,2), tracks_ends(:,2), tracks_heights(:,2))  ;    
    camera_extrinsics__latency_between_track_points_as_ratio()
    camera_extrinsics__latency_between_track_points_as_ratio()
    points_3D_with_latency_1 = camera_extrinsics__latency_between_track_points_as_ratio(points_3D_track_1 , tracks_heights(1,:) , 0.01);
    points_3D_with_latency_2 = camera_extrinsics__latency_between_track_points_as_ratio(points_3D_track_2 , tracks_heights(2,:) , 0.01);
    points_3D = horzcat(points_3D_with_latency_1,points_3D_with_latency_2);
else
    [ points_3D , points_3D_hom] = camera_extrinsics__generate_path_3D_data_sinusoid( 11 , 4 , [ 1 0.5 ] , [10 7] ,  [ 0.5  0.75  1.1 ] )
end



%---- 
%-- default camera : need this to get the camera_K at zero offset
camera_default = CentralCamera('default');
move_trans_x = [ 0 0 0]';
camera_default = camera_default.move(  [   [ eye(3), move_trans_x ] ; [ 0 0 0 1 ]   ]  );  % :-- move in world coordinate system ( FLU ) : camera is aligned to 
[ camera_K_default , Focal_length_default , Principal_point_default ] = camera_extrinsics__camera_intrinsics_from_pctoolkit_camera(camera_default);
%----
num_cam_poses = 3;
model_size = 0;
model_size_range=[4,8];
model_size_range=[8,9];
num_cam_poses = 1;                  %  HAVE TO change indexing if this is over 1  :  store models in vector along with parameters, or shift to objects/structures
num_RANSAC_iterations = 40;   % otten 100-1000, but papers imply can be significantly less 
num_RANSAC_iterations = 400;
inlier_threshold_pixel_diff = 6;
models = zeros(model_size,num_RANSAC_iterations, 'uint32');
models_max_diff_SE3_element = zeros(1,num_RANSAC_iterations);
models_pose = zeros(4,4,num_RANSAC_iterations);
models_best_model_candidates = zeros(1,num_RANSAC_iterations, 'uint32');
models_exceptions = zeros(1,num_RANSAC_iterations, 'uint32');
models_consensus_size = zeros(1,num_RANSAC_iterations, 'uint32');
cameras = [];
for jj_ = 1:num_cam_poses
    
    min_angle_degs=15; angle_range_degs=40; x_max=5; y_max=5; z_max=2; proportion_in_fov=1.0;    
    camera =  camera_extrinsics__place_camera_safely(min_angle_degs,angle_range_degs, x_max, y_max, z_max, points_3D_random, proportion_in_fov);
    
    %[ camera_K , Focal_length , Principal_point ] = camera_extrinsics__camera_intrinsics_from_pctoolkit_camera(camera);
    cameras = [ cameras ; camera ];
    
    points_2D = camera.project(points_3D_random);
    points_2D_without_noise = points_2D;
    pts_2D_noise_magnitude = 10;
    pts_2D_noise_mean = 0; 
    pixel_noise_type = 'None'
    if strcmpi(pixel_noise_type,'Gaussian' )
        points_2D_noise = camera_extrinsics__generate_noise_for_points_2D( size(points_2D,2) * 2,  pts_2D_noise_magnitude ,  pts_2D_noise_mean , 1 )    ;
        points_2D(1,:) = points_2D(1,:) + points_2D_noise(1,1:size(points_2D,2));    
        points_2D(2,:) = points_2D(2,:) + points_2D_noise(1,1+size(points_2D,2):size(points_2D,2)*2);    
    else
        points_2D_noise = zeros(size(points_2D));
        points_2D = points_2D + points_2D_noise;
    
    for model_size = [4]  %model_size_range(1):model_size_range(2)
        
           points_2D_preconditioned = points_2D;
           points_2D_preconditioned_without_noise = points_2D_without_noise;
           points_3D_random_preconditioned = points_3D_random;
           [ models , models_max_diff_SE3_element , models_extrinsic_estimate_as_local_to_world ] = ...  % , models_best_solution , models_solution_set ] = ...           
        camera_extrinsics__iterate_epnp  ( ...
            points_2D_preconditioned, points_3D_random_preconditioned, camera_K_default, ...
            num_RANSAC_iterations, model_size, ...
            camera.get_pose_transform);
        
        fig_2d_plot_points_2D_with_and_without_noise = figure('Name',sprintf('plot_points_2D_with_and_without_noise model errors-ish - model size %d',model_size)) ;hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z')
        hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');  plot([0 , 0 , 1024, 1024 , 0], [0 , 1024, 1024 , 0 , 0]) ;
        plot2_rows(points_2D_preconditioned_without_noise,'bd')
        for iii_ = 1:20; text(points_2D_preconditioned_without_noise(1,iii_)+10,points_2D_preconditioned_without_noise(2,iii_)+10.,sprintf('%d',iii_),'Color','b');end
        plot2_rows(points_2D_preconditioned,'rx')
        for iii_ = 1:20; text(points_2D_preconditioned(1,iii_)+10,points_2D_preconditioned(2,iii_)+10.,sprintf('%d',iii_),'Color','r');end
        
        fig_2d_plot_errors_handle = figure('Name',sprintf('model errors-ish - model size %d',model_size)) ;plot(models_max_diff_SE3_element );
        % plot 3D
        fig_3d_handle = figure('Name',sprintf( 'points_3D_random - model size %d' ,model_size) ); hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z')
        camera_extrinsics__plot_3d_point_3D_and_camera_pose(fig_3d_handle, points_3D_random,camera)    
        for iii_ = 1:20; text(points_3D_random_preconditioned(1,iii_)+.05,points_3D_random_preconditioned(2,iii_)+.05,points_3D_random_preconditioned(3,iii_)+.05, sprintf('%d',iii_));end
        drawnow
        camera_extrinsics__plot_3d_estimated_poses   (fig_3d_handle, models_extrinsic_estimate_as_local_to_world)
        camera.plot_camera()
        drawnow
                
        display('now reproject');
        
        best_model_consensus_size = 0;
        for ii_ = 1:num_RANSAC_iterations   
%                 display(sprintf('RANSAC iteration %d',ii_));
%             try
                % now  project the 3D to 2D - PC style
                camera_estimated = CentralCamera('default');
                camera_estimated = camera_estimated.move(models_extrinsic_estimate_as_local_to_world(:,:,ii_));
                points_2D_estimated = camera_estimated.project(points_3D_random);
                reprojected_errs_uv( : , : , ii_  ) = points_2D_preconditioned - points_2D_estimated(1:2,:)  ;
                reprojected_errs_euc_sq(:,ii_) = reprojected_errs_uv( 1 , : , ii_  ).^2 + reprojected_errs_uv( 2 , : , ii_  ).^2;            
                reprojected_errs_euc(:,ii_) = reprojected_errs_euc_sq(:,ii_).^0.5;
                models_reprojected_errs_euc_sq_total( ii_ ) = sum(reprojected_errs_euc_sq(:,ii_));
                inlier_points_2D_estimated(:,ii_) = reprojected_errs_euc(:,ii_)<= inlier_threshold_pixel_diff;
                models_consensus_size(ii_) = sum(inlier_points_2D_estimated(:,ii_));
                draw_2D_per_RANSAC = false;
                if draw_2D_per_RANSAC
                    figure('Name', ...
                        sprintf( 'points_2D - model size %d, model %d, sum(err_sq)=%f' ,model_size, ii_,models_reprojected_errs_euc_sq_total(ii_))) ; 
                        hold on;       axis equal;   plot([0 , 0 , 1024, 1024 , 0], [0 , 1024, 1024 , 0 , 0]) ;
                            kx_handle = plot2_rows(points_2D_preconditioned_without_noise,'kx');
                            bx_handle = plot2_rows(points_2D_preconditioned, 'bx') ;        
                            rx_handle = plot2_rows(points_2D_estimated(1:2,:) , 'rx') ;
                            bo_handle = plot2_rows(points_2D_preconditioned, 'bo') ;        
                            ro_handle = plot2_rows(points_2D_estimated(1:2,:) , 'ro') ;
                            inlier_handle = plot2_rows(points_2D_estimated(1:2,inlier_points_2D_estimated(:,ii_)>0) , 'ks', 'Linewidth',3 );
                            for iii_ = 1:20; text(points_2D_preconditioned(1,iii_)+10,points_2D_preconditioned(2,iii_)+10.,sprintf('%d',iii_));end
                            legend([kx_handle,bx_handle,rx_handle, inlier_handle], {'points 2D  zero noise',sprintf('points 2D with noise mag=%2.2f, mean=%2.2f',pts_2D_noise_magnitude,pts_2D_noise_mean),'reprojected', 'inliers'});
                            drawnow
                end
%             catch ME
%                 display(sprintf('------------------------\nexception with ii_=%d : %s',ii_,ME.getReport()))
%                 models_exceptions(ii_) = 1;
%             end            
            best_model_consensus_size = max(models_consensus_size);
            models_best_model_candidates = models_consensus_size>=best_model_consensus_size-1;
            models_best_by_consensus_size = find(models_best_model_candidates);
            models_best_model_candidates_data_points = models( : , models_best_by_consensus_size);
            models_best_by_consensus_size__min_reproj = min ( models_reprojected_errs_euc_sq_total(models_best_by_consensus_size) ) ;
            models_best_by_consensus_size_and_reproj= models_reprojected_errs_euc_sq_total<=models_best_by_consensus_size__min_reproj ;
        end                
                pooled_model_datapoint_indices_of_best_models = unique(reshape(models(:,models_best_model_candidates), 1 , [] ))  ;
                sum( inlier_points_2D_estimated(:,models_best_model_candidates) , 2 )  ;
                
        % re-estimate EPnP using the inliers of the best models by consensus set size and reprojection error 
        display('re-estimate EPnP using the inliers of the best models by consensus set size and reprojection error ')
        points_3D_inliers_of_best_model = points_3D_random_preconditioned(:,sum(inlier_points_2D_estimated(:,models_best_by_consensus_size_and_reproj),2)>0)  ;
        points_2D_inliers_of_best_model = points_2D_preconditioned(:,sum(inlier_points_2D_estimated(:,models_best_by_consensus_size_and_reproj),2)>0)  ;
        
        num_RANSAC_iterations_1 = 1; model_size_1 = models_consensus_size(models_best_by_consensus_size_and_reproj);
           [ models , models_max_diff_SE3_element , models_extrinsic_estimate_as_local_to_world_1 ] = ...  % , models_best_solution , models_solution_set ] = ...           
        camera_extrinsics__iterate_epnp  ( ...
            points_2D_inliers_of_best_model, points_3D_inliers_of_best_model, camera_K_default, ...
            num_RANSAC_iterations_1, model_size_1, ...
            camera.get_pose_transform);
        
                best_camera_estimated = CentralCamera('default');
                best_camera_estimated = best_camera_estimated.move(models_extrinsic_estimate_as_local_to_world_1);
                best_points_2D_estimated = best_camera_estimated.project(points_3D_random);
                best_reprojected_errs_uv( : , : , 1  ) = points_2D_preconditioned - best_points_2D_estimated(1:2,:)  ;
                best_reprojected_errs_euc_sq(:,1) = best_reprojected_errs_uv( 1 , : , 1  ).^2 + best_reprojected_errs_uv( 2 , : , 1  ).^2;            
                best_reprojected_errs_euc(:,1) = best_reprojected_errs_euc_sq(:,1).^0.5;
                best_models_reprojected_errs_euc_sq_total( 1 ) = sum(best_reprojected_errs_euc_sq(:,1));
                best_inlier_points_2D_estimated(:,1) = best_reprojected_errs_euc(:,1)<= inlier_threshold_pixel_diff;
                best_models_consensus_size(1) = sum(best_inlier_points_2D_estimated(:,1));
                figure('Name', ...
                    sprintf( 'Estimated on all inliers of best model points_2D - model size %d, model %d, sum(err_sq)=%f' ,model_size, 1,best_models_reprojected_errs_euc_sq_total(1))) ; 
                    hold on;       axis equal;   plot([0 , 0 , 1024, 1024 , 0], [0 , 1024, 1024 , 0 , 0]) ;
                        kx_handle = plot2_rows(points_2D_preconditioned_without_noise,'kx');
                        bx_handle = plot2_rows(points_2D_preconditioned, 'bx') ;        
                        rx_handle = plot2_rows(best_points_2D_estimated(1:2,:) , 'rx') ;
                        bo_handle = plot2_rows(points_2D_preconditioned, 'bo') ;        
                        ro_handle = plot2_rows(best_points_2D_estimated(1:2,:) , 'ro') ;
                        inlier_handle = plot2_rows(best_points_2D_estimated(1:2,best_inlier_points_2D_estimated(:,1)>0) , 'ks', 'Linewidth',3 );
                        for iii_ = 1:20; text(points_2D_preconditioned(1,iii_)+10,points_2D_preconditioned(2,iii_)+10.,sprintf('%d',iii_));end
                        legend([kx_handle,bx_handle,rx_handle, inlier_handle], {'points 2D  zero noise',sprintf('points 2D with noise mag=%2.2f, mean=%2.2f',pts_2D_noise_magnitude,pts_2D_noise_mean),'reprojected', 'inliers'});
                        drawnow
                        
            % re-estimate EPnP with least squares optimisation, using the inliers of the best model by consensus set size and reprojection error 
                    %       x3d_h: homogeneous coordinates of the points in world reference
                    %       x2d_h: homogeneous position of the points in the image plane
                    %       A: intrincic camera parameters
    %{   
    throwing errors? inside efficient_pnp_gauss>reprojection_error_usingRT (line 282)                   
            [R_gauss,T_gauss,Xc_gauss,best_solution_gauss,opt_gauss]=efficient_pnp_gauss( ...
                geo__euclidean_3D_to_hom(points_3D_inliers_of_best_model)' , ...
                geo__euclidean_2D_to_hom(best_points_2D_estimated(:,best_reprojected_errs_euc(:,1)<= inlier_threshold_pixel_diff))' , ...
                camera_K_default)
            extrinsic_estimate_as_local_to_world_gauss =  vertcat(  horzcat( R_gauss', (-(R_gauss')) * T_gauss )  ,   [ 0 0 0 1]  );    
            models_extrinsic_estimate_as_local_to_world_gauss(:,:,ii_) = extrinsic_estimate_as_local_to_world_gauss;
        
                best_camera_estimated_gauss = CentralCamera('default');
                best_camera_estimated_gauss = best_camera_estimated_gauss.move(models_extrinsic_estimate_as_local_to_world_1);
                best_points_2D_estimated_gauss = best_camera_estimated_gauss.project(points_3D_random);
                best_reprojected_errs_uv_gauss( : , : , 1  ) = points_2D_preconditioned - best_points_2D_estimated_gauss(1:2,:)  ;
                best_reprojected_errs_euc_sq_gauss(:,1) = best_reprojected_errs_uv_gauss( 1 , : , 1  ).^2 + best_reprojected_errs_uv_gauss( 2 , : , 1  ).^2;            
                best_reprojected_errs_euc_gauss(:,1) = best_reprojected_errs_euc_sq_gauss(:,1).^0.5;
                best_models_reprojected_errs_euc_sq_total_gauss( 1 ) = sum(best_reprojected_errs_euc_sq_gauss(:,1));
                best_inlier_points_2D_estimated_gauss(:,1) = best_reprojected_errs_euc_gauss(:,1)<= inlier_threshold_pixel_diff;
                best_models_consensus_size_gauss(1) = sum(best_inlier_points_2D_estimated_gauss(:,1));
                figure('Name', ...
                    sprintf( 'BEST + Gaussian points_2D - model size %d, model %d, sum(err_sq)=%f' ,model_size, 1,best_models_reprojected_errs_euc_sq_total_gauss(1))) ; 
                    hold on;       axis equal;   plot([0 , 0 , 1024, 1024 , 0], [0 , 1024, 1024 , 0 , 0]) ;
                        kx_handle = plot2_rows(points_2D_preconditioned_without_noise,'kx');
                        bx_handle = plot2_rows(points_2D_preconditioned, 'bx') ;        
                        rx_handle = plot2_rows(best_points_2D_estimated_gauss(1:2,:) , 'rx') ;
                        bo_handle = plot2_rows(points_2D_preconditioned, 'bo') ;        
                        ro_handle = plot2_rows(best_points_2D_estimated_gauss(1:2,:) , 'ro') ;
                        inlier_handle = plot2_rows(best_points_2D_estimated_gauss(1:2,best_inlier_points_2D_estimated_gauss(:,1)>0) , 'ks', 'Linewidth',3 );
                        for iii_ = 1:20; text(points_2D_preconditioned(1,iii_)+10,points_2D_preconditioned(2,iii_)+10.,sprintf('%d',iii_));end
                        legend([kx_handle,bx_handle,rx_handle, inlier_handle], {'points 2D  zero noise',sprintf('points 2D with noise mag=%2.2f, mean=%2.2f',pts_2D_noise_magnitude,pts_2D_noise_mean),'reprojected', 'inliers'});
                        drawnow
        %}
        camera_K_hom_RDF = [ camera_K_default , [ 0 0 0 ]'  ] ;
    end    
    
    
%     end_pt_yaw_then_pitch_rotation =  (roty(pitch_down_rads)) * rotz(yaw_left_rads)
%     end_pt_yaw_then_pitch_transform = [ [ end_pt_yaw_then_pitch_rotation ; 0 0 0 ] , camera_position ] 
%     end_pt_yaw_then_pitch = end_pt_yaw_then_pitch_transform * geo__euclidean_3D_to_hom(adj*eye(3))
%     plot3_rows( end_pt_yaw_then_pitch(:,1) , 'co', 'Linewidth',3)   % check the angle up
%     draw_axes_direct_c(end_pt_yaw_then_pitch(3), camera_position(1:3), 'cam_position not rot',adj,'c');
%     draw_axes_direct(end_pt_yaw_then_pitch(3), camera_position(1:3), 'cam_position not rot', 0.75*adj);

    % camera position, with axes aligned with world coordinate frame:  
    plot3_rows( camera_position , 'ms', 'Linewidth',3)
    draw_axes_direct_c(eye(3), camera_position(1:3), 'cam_position not rot',adj,'c');
    draw_axes_direct(eye(3), camera_position(1:3), 'cam_position not rot', 0.75*adj);
    
end




end