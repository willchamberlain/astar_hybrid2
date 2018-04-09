%{
Some (possibly flawed) analysis of the models found by RANSAC   in    iterate_sample_cam_605_pnp_reproject_2
%}

addpath('/mnt/nixbig/ownCloud/project_code'); 
addpath('/mnt/nixbig/ownCloud/project_code/graphics'); 
addpath('/mnt/nixbig/ownCloud/project_code/reprojection'); 
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

%%
% Run estimation
[ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx, reprojected_errs_euc_hist ] ...
        = iterate_sample_cam_605_pnp_reproject_2(200,8,1);
%    = iterate_sample_cam_605_pnp_reproject_2(10,8,1);
%     = iterate_sample_cam_605_pnp_reproject_2(50,8,1);

%%
% goodness of points  :  bigger is gooder
goodness_of_points_  = sum ( reprojected_errs_euc_hist(:,:) < 20 , 1 )
goodness_of_points_  = goodness_of_points(reprojected_errs_euc_hist , 20)
mean(goodness_of_points_)
std(goodness_of_points_)
prctile(goodness_of_points_,[2.5 25 50 75 97.5])

% goodness of models  :  bigger is gooder
goodness_of_models_ = sum ( reprojected_errs_euc_hist(:,:) < 20 , 2 )
goodness_of_models_ = goodness_of_models(reprojected_errs_euc_hist, 20)

figure('Name','goodness_of_models_')
plot( goodness_of_models_ ); pause;
histogram( goodness_of_models_); 
mean(goodness_of_models_)
std(goodness_of_models_)
prctile(goodness_of_models_,[0 2.5 25 50 75 97.5 100])
histogram( goodness_of_models_, [ prctile(goodness_of_models_,[0 2.5 25 50 75 97.5 100]) max(goodness_of_models_)  ])
% nope - is bivariate - want to plot side-by-side for various thresholds histogram2( sum ( reprojected_errs_euc_hist(:,:) < 20  ))

% for now - 
goodness_of_models_ = goodness_of_models(reprojected_errs_euc_hist, 10)
histcounts(goodness_of_models_ , [1:24] )

% work out which models have the best - ! don't need to automate this right now !
prctile( [1:24] , [10:10:100])
histcounts(goodness_of_models_ , prctile( [1:24] , [10:10:100]) )
histcounts(goodness_of_models_ , [1:2:24] )

% do want to find the set of datapoints that are most consistent and get a model off of them - the consensus set
% 1) union of consensus sets of datapoints of the best models 
% 2) intersection of consensus sets of datapoints of the best models 
% 3) ?? weighted union - 1 vote per occurance 
%   - I think that I can do that here by duplicating points :- may have to give them an 
%     infestiminal offset if there's some duplicate rejection
goodness_of_points_  = goodness_of_points ( reprojected_errs_euc_hist  , 5 )
for ii_ = 1:20
    goodness_of_points_surf(:,ii_) = goodness_of_points ( reprojected_errs_euc_hist  , ii_ );
end
surf([1:20],[1:24],goodness_of_points_surf)
surfc([1:20],[1:24],goodness_of_points_surf); xlabel('threshold'); ylabel('datapoint#'); axis equal; hold on;


plot3( [3 1 1 1 8 ] , [2 2 18 18  23 ] , [4 4 5 8 10] , 'r')    % draw arbitrary structures 

        % draw a histogram 
%  fill3( [ 0 0 0 0]  , [ 4.5 4.5 5.5 5.5 ] , [ 0 goodness_of_points_surf(10,5) goodness_of_points_surf(10,5) 0 ] , 'r')
%   addpath('/mnt/nixbig/ownCloud/project_code/graphics/')
%   flat_histogram_for_3d_charts.m

for  threshold = 7:-1:1 %20:-2:2
    for ii_ = 1:24
        fill3( [ 0 0 0 0]  , [ ii_-0.5 ii_-0.5 ii_+0.5 ii_+0.5 ] , [ 0 goodness_of_points_surf(ii_,threshold) goodness_of_points_surf(ii_,threshold) 0 ] , [ threshold/7    0   1- (threshold/7) ])
    end
end

%  draw the good and bad subsets of data points - look for anything obvious 
%  points are in   /mnt/nixbig/ownCloud/project_code/camera_extrinsics_data_analysis__iterations.m
goodness_of_points_  = goodness_of_points ( reprojected_errs_euc_hist  , 1.5 )   ;
goodness_of_points_  = goodness_of_points ( reprojected_errs_euc_hist  , 5 )   ;
goodness_of_points_idx = find(goodness_of_points_)   ;
badness_of_points_idx    = setdiff( [1:24] , find(goodness_of_points_) )   ;
figure
hold on; axis equal; grid on;
plot3( 0 , 0 , 0 , 'bo')
plot3_rows(points_3D_reported_FLU(:,goodness_of_points_idx) ,'rx')
plot3_rows(points_3D_reported_FLU(:,badness_of_points_idx) ,'kd')


% Re-run estimation on the consensus set
[ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx, reprojected_errs_euc_hist ] ...
    = iterate_sample_cam_605_pnp_reproject_2(50,4,1 ,  points_2D_reported(:,goodness_of_points_idx), points_3D_reported_FLU(:,goodness_of_points_idx));

[ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx, reprojected_errs_euc_hist ] ...
    = iterate_sample_cam_605_pnp_reproject_2(200,4,1 ,  points_2D_reported(:,goodness_of_points_idx), points_3D_reported_FLU(:,goodness_of_points_idx));


world_to_camera_rotm_hist_good = world_to_camera_rotm_hist ;
world_to_camera_trans_hist_good = world_to_camera_trans_hist ;
best_err_tukey_100_100_idx_good = best_err_tukey_100_100_idx ;
best_err_tukey_50_50_idx_good = best_err_tukey_50_50_idx ;
reprojected_errs_euc_hist_good = reprojected_errs_euc_hist ;
goodness_of_models__good = goodness_of_models_;

% Re-run estimation on the consensus set
[ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx, reprojected_errs_euc_hist ] ...
    = iterate_sample_cam_605_pnp_reproject_2(20,6,1 ,  points_2D_reported(:,goodness_of_points_idx), points_3D_reported_FLU(:,goodness_of_points_idx));


goodness_of_models_ = sum ( reprojected_errs_euc_hist(:,:) < 20 , 2 )
goodness_of_models_ = goodness_of_models(reprojected_errs_euc_hist, 20)

goodness_of_models_ = sum ( reprojected_errs_euc_hist(:,:) < 10 , 2 )
goodness_of_models_ = goodness_of_models(reprojected_errs_euc_hist, 10)
goodness_of_models_ = goodness_of_models(reprojected_errs_euc_hist, 5)
 sum((goodness_of_models_>0))

%  go to    /mnt/nixbig/ownCloud/project_code/camera_extrinsics_data_analysis_try_fitting_gaussian.m

threshold = 2;
% goodness of points  :  bigger is gooder
goodness_of_points2_  = sum ( reprojected_errs_euc_hist(:,:) < threshold , 1 )
goodness_of_points2_  = goodness_of_points(reprojected_errs_euc_hist , threshold)
mean(goodness_of_points2_)
std(goodness_of_points2_)
prctile(goodness_of_points2_,[2.5 25 50 75 97.5])

% goodness of models  :  bigger is gooder
goodness_of_models2_ = sum ( reprojected_errs_euc_hist(:,:) < threshold , 2 )
goodness_of_models2_ = goodness_of_models(reprojected_errs_euc_hist, threshold)
sum(goodness_of_models2_)  % 15/20 


