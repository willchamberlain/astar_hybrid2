best_by_reprojection_inliers_1 = find(reprojection_inliers_1(:)==max(reprojection_inliers_1))  ;

    points_2D_reprojected_inliers = zeros( [ size(points_2D) num_RANSAC_iterations ] )  ;  % 3xnum_datapointsxnum_RANSAC_iterations      
    reprojection_difference_inliers = zeros( [ size(points_2D) num_RANSAC_iterations ] )  ;  % 3xnum_datapointsxnum_RANSAC_iterations  
    reprojection_Euclidean_inliers = zeros( [ 1 size(points_2D,2) num_RANSAC_iterations ] )  ;  %  1xnum_datapointsxnum_RANSAC_iterations
    reprojection_Euclidean_total_inliers = zeros( [ 1 num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
    reprojection_inliers_2_inliers = zeros( [ 1 num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations
    reprojection_inliers_1_inliers = zeros( [ 1 num_RANSAC_iterations ] )  ;  %  1xnum_RANSAC_iterations

for jj_ = 1:size(best_by_reprojection_inliers_1,1)       
    
        ii_ = best_by_reprojection_inliers_1(jj_)  ;
        
            model_size_linliers = max(reprojection_inliers_1); 
            num_RANSAC_iterations = 100;   % otten 100-1000, but papers imply can be significantly less 
           [ models_linliers, models_max_diff_SE3_element_linliers , models_extrinsic_estimate_as_local_to_world_linliers ] = ...  % , models_best_solution , models_solution_set ] = ...           
        camera_extrinsics__iterate_epnp  ( ...
            points_2D_preconditioned(:,reprojection_Euclidean(:,:,ii_)<=1), points_3D_preconditioned(:,reprojection_Euclidean(:,:,ii_)<=1), camera_K_default, ...
            1, model_size_linliers, ...
            camera.get_pose_transform);    
    
        pose_estimate = squeeze( models_extrinsic_estimate_as_local_to_world_linliers(:,:, 1) )  ;
        cam_reproject = CentralCamera('default')  ;  cam_reproject.T = pose_estimate  ;  
%         points_2D_reprojected(:,:,ii_) = cam_reproject.project( points_3D_preconditioned )  ;
        points_2D_reprojected_inliers(:,reprojection_Euclidean(:,:,ii_)<=1,jj_) = cam_reproject.project( points_3D_preconditioned_no_latency(:,reprojection_Euclidean(:,:,ii_)<=1) )  ;        
        reprojection_difference_inliers(:,:,jj_) = points_2D_reprojected_inliers(:,:,jj_) - points_2D  ;
        reprojection_Euclidean_inliers(:,:,jj_) = norm_2(reprojection_difference_inliers(:,:,jj_),1)  ;
        reprojection_Euclidean_total_inliers(jj_) = sum(reprojection_Euclidean_inliers(:,:,jj_))  ;
        reprojection_inliers_1_inliers(jj_) = sum(reprojection_Euclidean_inliers(:,:,jj_)<=1)   ;
        reprojection_inliers_2_inliers(jj_) = sum(reprojection_Euclidean_inliers(:,:,jj_)<=2)   ;
end        

reprojection_inliers_1_inliers(reprojection_inliers_1_inliers>1)
reprojection_Euclidean_total_inliers(reprojection_inliers_1_inliers>1)

reprojection_Euclidean_total_inliers(reprojection_Euclidean_total_inliers(:)>0)

