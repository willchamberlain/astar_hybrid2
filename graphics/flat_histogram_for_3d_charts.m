addpath('/mnt/nixbig/ownCloud/project_code'); 
addpath('/mnt/nixbig/ownCloud/project_code/graphics'); 
addpath('/mnt/nixbig/ownCloud/project_code/reprojection'); 
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

threshold = 5;
for ii_ = 1:24
    fill3( [ 0 0 0 0]  , [ ii_-0.5 ii_-0.5 ii_+0.5 ii_+0.5 ] , [ 0 goodness_of_points_surf(ii_,threshold) goodness_of_points_surf(ii_,threshold) 0 ] , 'b')
end