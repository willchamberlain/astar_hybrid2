%{
Ground truth is from canned values  : 
        cam_605_good
    variables from fixed array of data

Can use canned data : sets             
        points_2D_reported
        points_3D_reported_FLU
    variables from fixed arrays of data if not passed as parameters

Uses canned camera extrinsics and distortion parameters : 
------------------------------------------------------------------


cam_605_good = [...
    0.887598151824310  -1.97885166499638  0.957895776523009  -0.0318571289684177  0.187864426800859  0.248082165295458  0.949814360672122
    ];
world_to_camera_groundtruth_rotm =  quat2rotm( [ cam_605_good(1,7) , cam_605_good(1,4:6) ] );
world_to_camera_groundtruth_trans = cam_605_good(1,1:3)';
world_to_camera_groundtruth_ = [  ...
    [world_to_camera_groundtruth_rotm , world_to_camera_groundtruth_trans]  ;
    0 0 0 1 ]  ;
---
FLU_basis_vectors = eye(4);   
T_FLU_to_RDF = [ ...
     0    -1     0     0
     0     0    -1     0
     1     0     0     0
     0     0     0     1   ] ;
RDF_basis_vectors = FLU_basis_vectors * T_FLU_to_RDF;

camera_K = [...
   322.9596901156589 , 000.0000000000000 , 176.8267919600727 ; %,    //  f_x ,   0 , c_x
                000.0000000000000 , 323.8523693059909 , 146.7681514313797 ; %,    //    0 , f_y , c_y
                  0.0 ,               0.0 ,               1.0 ];            
camera_K_hom_RDF = [ camera_K , [ 0 0 0 ]'  ] ;

Radial          =    [0.004180016841640, 0.136452931271259, -0.638647134308425];
Tangental   =   [-0.001666231998527, -0.00008160213039217031];
%Focal_length =  [3.229596901156589e+02, 3.238523693059909e+02];
%Principal_point =  [1.768267919600727e+02, 1.467681514313797e+02];
Focal_length =  [camera_K(1,1) , camera_K(2,2)]  ;
Principal_point =  [ camera_K(1,3) , camera_K(2,3) ]  ;

------------------------------------------------------------------


ransac_over_epnp.m (
	2D dataset - undistorted ('conditioned')
	3D dataset
	model size = number of datapoints needed to estimate a model e.g. 4
	datapoint_inlier_threshold = reprojection error threshold = threshold to determine whether datapoint is an inlier for a model e.g. 10pixels 
	number_of_iterations = number of models formed: if all models are valid, the number of RANSAC iterations
	)
	best_model
	(best_model_refined  -  LATER: refine model using least-squared / some optimisation vs whole inlier set)
	best_model_inlier_set
	best_model_inlier_set_size 
	best_model_inlier_set_reprojection_error
	models - indices into the sample dataset
	(models_refined  -  LATER: refine model using least-squared / some optimisation vs whole inlier set)
	models_inlier_set   - indices into the sample dataset, including the model datapoints
	models_inlier_set_size 
	models_inlier_set_reprojection_error - sum over the inlier's reprojection errors

%}
function [ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx, reprojected_errs_euc_hist ] = ...
    camera_extrinsics__ransac_over_epnp ( ...
    data_2d_undistorted_hom, data_3d_hom, ...
    model_size, ...
    datapoint_inlier_threshold, ...
    number_of_iterations, ...
    draw_axes_)

addpath('/mnt/nixbig/ownCloud/project_code'); 
addpath('/mnt/nixbig/ownCloud/project_code/graphics'); 
addpath('/mnt/nixbig/ownCloud/project_code/reprojection'); 
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

    
    dataset_size=size(data_2d_undistorted_hom,1);
	models = boolean(zeros(dataset_size,number_of_iterations, 'uint32'));  
	models_valid = boolean(zeros(number_of_iterations, 'uint32'));  
	models_inlier_set = boolean(zeros(dataset_size,number_of_iterations, 'uint32'));  
	models_inlier_set_size = zeros(number_of_iterations, 'uint32');
	models_inlier_set_reprojection_error = zeros(number_of_iterations,'double');
	best_model = boolean(zeros(dataset_size,'uint32'));  
	best_model_inlier_set = boolean(zeros(dataset_size,'uint32'));  
	best_model_inlier_set_size = 0;
	best_model_inlier_set_reprojection_error = inf;
    
    for ii_ = 1:number_of_iterations
        %     draw a random subset of the dataset as the model
        [sampled_values,index_vector] = datasample( data_2d_undistorted_hom, model_size, 'Replace',false);
        models( [index_vector] , ii_) = true;
        
        
    end

    
estimates = zeros(num_iternations,3);
err = zeros(num_iternations,1);
size(points_2D_reported)
world_to_camera_rotm_hist = zeros(num_iternations, 3, 3);
world_to_camera_trans_hist = zeros(num_iternations, 3, 1);
reprojected_errs = zeros(num_iternations, size(points_2D_reported,1), size(points_2D_reported,2));
reprojected_errs_bad = zeros(num_iternations, size(points_2D_reported,2));
best_err = 1000000*2*size(points_2D_reported,2)   ;
best_err_idx = -1   ;
best_err_tukey_100_100 = 1000000*2*size(points_2D_reported,2)   ;
best_err_tukey_100_100_idx = -1   ;
best_err_tukey_50_50 = 1000000*2*size(points_2D_reported,2)   ;
best_err_tukey_50_50_idx = -1   ;

% best_err_tukey_100_0 = 1000000*2*size(points_2D_reported,2)   ;
% best_err_tukey_100_0_idx = -1   ;

world_to_camera_est_FLU_best=1;  world_to_local_FLU_best=1;  est_rotm_FLU_best=1;
for ii_ = 1:num_iternations 
    try
    %-- estimation    
    [aa , bb , cc  , world_to_camera_est_FLU, world_to_local_FLU,  est_rotm_FLU , solution_set , Alph ] = func_cam_605_select_points_input( camera_K , Radial , Tangental , points_2D_reported , points_3D_reported_world_FLU(1:3,:)  , num_data_points_per_sample , split_z , draw_axes_ );
        %-- estimation with mix of sample sizes - e.g. using 8 and 4 points 
        %  if mod(ii_,2)==0 && num_points >= num_points_min*2
        %       [aa , bb , cc  , world_to_camera_est_FLU, world_to_local_FLU,  est_rotm_FLU . solution_set , Alph ] = func_cam_605_select_points_input( camera_K , Radial , Tangental , points_2D_reported , points_3D_reported_world_FLU(1:3,:)  , num_points/2 , split_z , draw_axes_ );
        %  else
        %       [aa , bb , cc  , world_to_camera_est_FLU, world_to_local_FLU,  est_rotm_FLU . solution_set , Alph ] = func_cam_605_select_points_input( camera_K , Radial , Tangental , points_2D_reported , points_3D_reported_world_FLU(1:3,:)  , num_points , split_z , draw_axes_ );
        %  end
%     display(world_to_camera_est_FLU)
    if draw_axes_
        draw_axes_direct(world_to_camera_est_FLU(1:3,1:3), world_to_camera_est_FLU(1:3,4)+0.2 , '', 1.2 )
%         draw_axes_direct_c(world_to_camera_est_FLU(1:3,1:3), world_to_camera_est_FLU(1:3,4)+0.2 , strrep('world_to_camera_est_FLU','_','\_'), 0.6, 'k' )
        draw_axes_direct_c(world_to_camera_est_FLU(1:3,1:3), world_to_camera_est_FLU(1:3,4)+0.2 , '' , 0.6, 'k' )
    end;
    estimates(ii_,:) = [aa bb cc];
    err(ii_) = sqrt(sum(([0.887598151824310  -1.97885166499638  0.957895776523009] -  [aa bb cc] ).^2));        
    %-- reprojection                       


    %     points_3D_reported_world_RDF = T_FLU_to_RDF * vertcat(points_3D_reported, ones( 1 ,  size(points_3D_reported, 2) ) ) ;    
    world_to_camera_rotm  = world_to_camera_est_FLU(1:3 , 1:3) ;     world_to_camera_rotm_hist( ii_ , : , :) = world_to_camera_rotm ;
    world_to_camera_trans = world_to_camera_est_FLU(1:3 , 4) ;        world_to_camera_trans_hist( ii_ , : , :) = world_to_camera_trans ;
    
    if 0
        figure('Name','points_3D_reported_world_FLU');   hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
        plot3_rows(points_3D_reported_world_FLU, 'rx'); axis equal; grid on        
        plot3(0,0,0,'bo')        
        for ii_ = 1 : size(points_3D_reported_world_FLU,2)
            text( points_3D_reported_world_FLU(1,ii_) , points_3D_reported_world_FLU(2,ii_) , points_3D_reported_world_FLU(3,ii_) , sprintf('%d', ii_) );
        end
        draw_axes_direct(world_to_camera_est_FLU_rot, world_to_camera_est_FLU_t, '',1)
        
        % -- looks wrong ????
        figure('Name','points_3D_reported_local_FLU');   hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
        plot3_rows(points_3D_reported_local_FLU, 'rx');   
        plot3(0,0,0,'bo')           
        for ii_ = 1 : size(points_3D_reported_local_FLU,2)
            text( points_3D_reported_local_FLU(1,ii_) , points_3D_reported_local_FLU(2,ii_) , points_3D_reported_local_FLU(3,ii_) , sprintf('%d', ii_) );
        end        
    end
    
    
%     display(world_to_local_FLU);
    world_to_local_FLU = [ [ world_to_camera_rotm' , world_to_camera_rotm' * world_to_camera_trans .* -1]  ;  [ 0 0 0 1 ] ]  ;
%     display(world_to_local_FLU);      %   tr_invert(world_to_camera_est_FLU)
    
    points_3D_reported_local_FLU =  world_to_local_FLU * points_3D_reported_world_FLU ;
    points_3D_reported_local_RDF = T_FLU_to_RDF * points_3D_reported_local_FLU ;
    
    reprojected_ = camera_K_hom_RDF  *  points_3D_reported_local_RDF    ;
    
    reprojected__ = reprojected_ ;
    
    reprojected__(1,:)= reprojected_(1,:).* reprojected_(3,:).^-1  ;
    reprojected__(2,:)= reprojected_(2,:).* reprojected_(3,:).^-1  ;
    reprojected__(3,:)= reprojected_(3,:).* reprojected_(3,:).^-1  ;
    
    [u_undistorted,v_undistorted] = ...
    undistort_pinhole(  ...
        Radial , Tangental , Principal_point , Focal_length , ...         %  check v vs principal point - up or down?  
        reprojected__(1,:) , reprojected__(2,:)   ...                               %  check v vs principal point - up or down? 
        );
    reprojected__ = vertcat(  u_undistorted,  v_undistorted, reprojected__(3,:)  );    %  check v vs principal point - up or down? 
    
    reprojected_errs( ii_ , : , : ) = abs(points_2D_reported - reprojected__(1:2,:) )  ;
    reprojected_errs_bad( ii_ , : )  =  reprojected__( 1 , : ) < 0  |  reprojected__( 2 , : ) < 0 ;
        
    reprojected_errs_sq = squeeze(reprojected_errs(ii_,:,:)).^2    ;
    reprojected_errs_euc = sqrt(sum(reprojected_errs_sq,1))   ;
    reprojected_errs_euc_hist(ii_,:) = reprojected_errs_euc';
    
    if best_err > sum(sum(reprojected_errs_euc ))
        best_err = sum(sum(reprojected_errs_euc ))  ;
        best_err_idx = ii_;
    end
    
    reprojected_errs_sq_tukey_100_100 = reprojected_errs_euc;
    reprojected_errs_sq_tukey_100_100(  reprojected_errs_sq_tukey_100_100  > 100 ) = 100;
    if best_err_tukey_100_100 > sum(sum( reprojected_errs_sq_tukey_100_100 ))
        best_err_tukey_100_100 = sum(sum( reprojected_errs_sq_tukey_100_100  ))  ;
        best_err_tukey_100_100_idx = ii_;
    end
    
    reprojected_errs_sq_tukey_50_50 = reprojected_errs_euc;
    reprojected_errs_sq_tukey_50_50(  reprojected_errs_sq_tukey_50_50  > 50 ) = 50;
    if best_err_tukey_50_50 > sum(sum( reprojected_errs_sq_tukey_50_50 ))
        best_err_tukey_50_50 = sum(sum( reprojected_errs_sq_tukey_50_50  ))  ;
        best_err_tukey_50_50_idx = ii_;
        world_to_camera_est_FLU_best = world_to_camera_est_FLU  ;
        world_to_local_FLU_best = world_to_local_FLU  ;
        est_rotm_FLU_best = est_rotm_FLU  ; 
    end
    
    % very vulnerable to outliers
%     reprojected_errs_sq_tukey_100_0 = reprojected_errs_sq;
%     reprojected_errs_sq_tukey_100_0(  reprojected_errs_sq_tukey_100_0  > 100 ) = 0;    
%     if best_err_tukey_100_0 > sum(sum( reprojected_errs_sq_tukey_100_0 ))
%         best_err_tukey_100_0 = sum(sum( reprojected_errs_sq_tukey_100_0  ))  ;
%         best_err_tukey_100_0_idx = ii_;
%     end
    
    %   for ii_ = 1:10 ;   bob(ii_,1) = sum(sum( squeeze(reprojected_errs_sq_tukey(ii_,:,:))  ));  end;
    catch
        display(sprintf('some sort of problem on iteration %d',ii_))   ;
    end
end

pose_bad = sum(reprojected_errs_bad);

if draw_axes_
    grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
    plot3_rows(points_3D_reported_world_FLU, 'rs');
    for ii_ = 1 : size(points_3D_reported_world_FLU,2) 
        text( points_3D_reported_world_FLU(1,ii_) , points_3D_reported_world_FLU(2,ii_) , points_3D_reported_world_FLU(3,ii_) , sprintf('%d', ii_) );
    end


    % plot the 3D observations and the best estimate pose 
    fig_h_points_3D_reported_world_FLU_best = figure3('points_3D_reported_world_FLU best');
    plot3_rows(points_3D_reported_world_FLU,'rx');   
    for ii_ = 1 : size(points_3D_reported_world_FLU,2)
        text( points_3D_reported_world_FLU(1,ii_) , points_3D_reported_world_FLU(2,ii_) , points_3D_reported_world_FLU(3,ii_) , sprintf('%d', ii_) );
    end
    plot3(0,0,0,'bo');  text( 0 , 0 , 0 , '\{O\}');
    axis_length = 0.8;
    draw_axes_direct(world_to_camera_est_FLU_best(1:3,1:3), world_to_camera_est_FLU_best(1:3,4) , '',  axis_length , 'LineWidth',2 )
    draw_axes_direct_c(world_to_camera_est_FLU_best(1:3,1:3), world_to_camera_est_FLU_best(1:3,4) , strrep('world_to_camera_est_FLU_best', '_', '\_'), axis_length*0.75 , 'r' )

    % draw_axes_direct(squeeze(world_to_camera_rotm_hist(best_err_tukey_100_0_idx,:,:)),squeeze(world_to_camera_trans_hist(best_err_tukey_100_0_idx,:,:))', strrep('best_err_tukey_100_0_idx','_','\_'), 0.5)
    draw_axes_direct(squeeze(world_to_camera_rotm_hist(best_err_tukey_100_100_idx,:,:)),squeeze(world_to_camera_trans_hist(best_err_tukey_100_100_idx,:,:))', strrep('best_err_tukey_100_100_idx','_','\_'), 0.5)
    draw_axes_direct(squeeze(world_to_camera_rotm_hist(best_err_tukey_50_50_idx,:,:)),squeeze(world_to_camera_trans_hist(best_err_tukey_50_50_idx,:,:))', strrep('best_err_tukey_50_50_idx','_','\_'), 0.5)
    draw_axes_direct(squeeze(world_to_camera_rotm_hist(best_err_idx,:,:)),squeeze(world_to_camera_trans_hist(best_err_idx,:,:))', strrep('best_err_idx','_','\_'), 0.5)

    % plot the known good pose 
    draw_axes_direct(world_to_camera_groundtruth_rotm,world_to_camera_groundtruth_trans,'',0.2)
end

return


%%
figure; hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
for ii_ = 1 :size(pose_bad,2)
    if pose_bad(ii_)        
        draw_axes_direct_c( squeeze(world_to_camera_rotm_hist(ii_,:,:)), world_to_camera_trans_hist(ii_,:,:)', 'BAD', 2,'k')
        plot3( world_to_camera_trans_hist(ii_,1,:),  world_to_camera_trans_hist(ii_,2,:),  world_to_camera_trans_hist(ii_,3,:),  'kd' , 'LineWidth', 3);
    else
        draw_axes_direct( squeeze(world_to_camera_rotm_hist(ii_,:,:)), world_to_camera_trans_hist(ii_,:,:)', 'GOOD', 1)
    end
end



%%
size(  squeeze(  reprojected_errs(:,1,:)   )  )

size(  squeeze(  sqrt(  reprojected_errs(:,1,:).^2 + reprojected_errs(:,2,:).^2  )  )  )

squeeze(  sqrt(  reprojected_errs(:,1,:).^2 + reprojected_errs(:,2,:).^2  )  )

squeeze(reprojected_errs(:,1,:)) < 0

reprojected_errs = abs( reprojected_errs );

reprojected__ < 0


%%
figure;hold on; grid on;
size(reprojected_errs)
for ii_ = 1:size(reprojected_errs,1) 
    % TODO - check that the points are in the right order 
    plot( ii_, sum(sum( squeeze( reprojected_errs( ii_ , : , : )  )  )) , 'rx')
end


%%
%  How many of the poses found are like the good pose?  
%  
% world_to_camera_trans_hist 
cam_605_good_trans = cam_605_good(1:3) ;
trans_errs = world_to_camera_trans_hist - repmat(cam_605_good_trans, size(world_to_camera_trans_hist,1) , 1 )  ;
trans_errs_rs = sqrt(trans_errs(:,1).^2+trans_errs(:,2).^2+trans_errs(:,3).^2)  ;
figure; plot(trans_errs_rs,'rx');
figure; hist(trans_errs_rs);
trans_errs_rs( trans_errs_rs< median(trans_errs_rs) )
figure; plot(  trans_errs_rs( trans_errs_rs< median(trans_errs_rs) )   , 'rx' );

%%
trans_errs_rs = sqrt( trans_errs(:,3).^2 )  ;
figure; plot(trans_errs_rs,'rx');
figure; hist(trans_errs_rs);
trans_errs_rs( trans_errs_rs< median(trans_errs_rs) )
figure; plot(  trans_errs_rs( trans_errs_rs< median(trans_errs_rs) )   , 'rx' );

%%
trans_errs_rs = sqrt( trans_errs(:,3).^2 )  ;
figure; plot3(  trans_errs(:,1) , trans_errs(:,2) , trans_errs(:,3) , 'rx');  grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
figure; plot3(  abs(trans_errs(:,1)) , abs(trans_errs(:,2)) , abs(trans_errs(:,3)) , 'rx');  grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
figure; hist(trans_errs_rs);
trans_errs_rs( trans_errs_rs< median(trans_errs_rs) )
figure; plot(  trans_errs_rs( trans_errs_rs< median(trans_errs_rs) )   , 'rx' );

%%  
%         Find 2D-3D points which give problems

apoint = [2, -1.1, 0.4 ]';
world_to_local_FLU = [ [ world_to_camera_good_rotm' , world_to_camera_good_rotm' * world_to_camera_good_trans .* -1]  ;  [ 0 0 0 1 ] ]

points_3D_reported_world_FLU =  vertcat(points_3D_reported_FLU, ones( 1 ,  size(points_3D_reported_FLU, 2) ) );
camera_K_hom_RDF = [ camera_K , [ 0 0 0 ]'  ];

camera_K_hom_RDF * T_FLU_to_RDF * [ 1 0 0  1 ]'

reprojected_ = camera_K_hom_RDF * T_FLU_to_RDF * world_to_local_FLU * points_3D_reported_world_FLU

reprojected__ = reprojected_; 
reprojected__(1,:)= reprojected_(1,:).* reprojected_(3,:).^-1
reprojected__(2,:)= reprojected_(2,:).* reprojected_(3,:).^-1
reprojected__(3,:)= reprojected_(3,:).* reprojected_(3,:).^-1
errs = points_2D_reported - reprojected__(1:2,:)
size(errs)
errs_sq = sqrt( errs(1,:).^2 + errs(2,:).^2 )
figure; plot( errs_sq )
figure; hist( errs_sq )
var( errs_sq )
std( errs_sq )
median( errs_sq )
mode( errs_sq )
mean( errs_sq )
median( errs_sq ) + std( errs_sq )
mode( errs_sq ) + std( errs_sq )

errs_sq > median( errs_sq ) + std( errs_sq )




%%

%{

cam_605_good = [...
    0.887598151824310  -1.97885166499638  0.957895776523009  -0.0318571289684177  0.187864426800859  0.248082165295458  0.949814360672122
    ];
apoint = [2, -1.1, 0.4 ]';
world_to_camera_good_rotm =  quat2rotm( [ cam_605_good(1,7) , cam_605_good(1,4:6) ] );
world_to_camera_good_trans = cam_605_good(1,1:3)';
figure;hold on; grid on; axis equal; xlabel('x'), ylabel('y'), zlabel('z');
draw_axes_direct(world_to_camera_good_rotm,world_to_camera_good_trans,'',0.2)
plot3( 0 , 0 , 0 , 'rx')
plot3( 2 , -3 , 0 , 'rx')
plot3_rows(points_3D_reported_world_FLU,'bo')
plot3(apoint(1),apoint(2),apoint(3),'bx')
% zlim( [ 0, 1.5 ] )

wc_ = vertcat (  horzcat( world_to_camera_good_rotm'  ,  world_to_camera_good_rotm' * ( -1*world_to_camera_good_trans )  )  ,  [ 0 0 0 1 ] )
wc_ = vertcat (  horzcat( world_to_camera_good_rotm  ,  world_to_camera_good_rotm * ( -1*world_to_camera_good_trans )  )  ,  [ 0 0 0 1 ] )
wc_ = vertcat (  horzcat( world_to_camera_good_rotm  ,  world_to_camera_good_trans  )  ,  [ 0 0 0 1 ] )

% world_to_camera_est_ = vertcat( horzcat( , )  ,  [  0  0  0  1  ]  )
%     reprojected_ = horzcat( camera_K , [ 0 ; 0 ; 0  ]) * world_to_camera_est_ * vertcat(points_3D_reported, ones( 1 ,  size(points_3D_reported, 2)  )  ) ;
    reprojected_ = horzcat( camera_K , [ 0 ; 0 ; 0  ]) * wc_ * vertcat(points_3D_reported_world_FLU, ones( 1 ,  size(points_3D_reported_world_FLU, 2)  )  ) ; 
    reprojected_ = horzcat( camera_K , [ 0 ; 0 ; 0  ]) * wc_ * vertcat(apoint, ones( 1 ,  size(apoint, 2)  )  ) ; 
    reprojected__ = reprojected_; 
    plot(reprojected__(1,:),reprojected__(2,:), 'gx')
    reprojected__(1,:)= reprojected_(1,:).* reprojected_(3,:).^-1
    reprojected__(2,:)= reprojected_(2,:).* reprojected_(3,:).^-1
    reprojected__(3,:)= reprojected_(3,:).* reprojected_(3,:).^-1
%     points_2D - reprojected__(1:2,:)
    figure;  hold on; grid on; axis equal; xlabel('x'), ylabel('y'), 
    plot(points_2D_reported(1,:),points_2D_reported(2,:), 'bo')
    plot(reprojected__(1,:),reprojected__(2,:), 'rx')


plot3(cam_605_good(:,1),cam_605_good(:,2),cam_605_good(:,3), 'ks');
grid on; axis equal; xlabel('x');ylabel('y');zlabel('z');

figure('Name','Estimates of Cam 605'); hold on; grid on; axis equal; xlabel('x');ylabel('y');zlabel('z');
plot3(estimates(:,1),estimates(:,2),estimates(:,3), 'rx');
plot3(cam_605_good(:,1),cam_605_good(:,2),cam_605_good(:,3), 'ks');

%}



end