%{
Ground truth is from canned values  : 
        cam_605_good
    variables from fixed array of data

Can use canned data : sets             
        points_2D_reported
        points_3D_reported_FLU
    variables from fixed arrays of data if not passed as parameters

Uses canned camera extrinsics and distortion parameters : 
%}
function [ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx, reprojected_errs_euc_hist ] = ...
    iterate_sample_cam_605_pnp_reproject_2(num_iternations, num_data_points_per_sample, draw_axes_, points_2d_arg, points_3d_arg)

addpath('/mnt/nixbig/ownCloud/project_code'); 
addpath('/mnt/nixbig/ownCloud/project_code/graphics'); 
addpath('/mnt/nixbig/ownCloud/project_code/reprojection'); 
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

cam_605_good = [...
    0.887598151824310  -1.97885166499638  0.957895776523009  -0.0318571289684177  0.187864426800859  0.248082165295458  0.949814360672122
    ];
world_to_camera_groundtruth_rotm =  quat2rotm( [ cam_605_good(1,7) , cam_605_good(1,4:6) ] );
world_to_camera_groundtruth_trans = cam_605_good(1,1:3)';
world_to_camera_groundtruth_ = [  ...
    [world_to_camera_groundtruth_rotm , world_to_camera_groundtruth_trans]  ;
    0 0 0 1 ]  ;
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
              
if ~exist('points_2d_arg','var') 
    points_2D_reported = [ ... 
        38.97217696070338 , 118.2824709789849 ;	32.034306321017944 , 192.466939933053 ;	93.31441891387527 , 96.54772083404143 ;	87.32955212333361 , 157.37000945678204 ;	126.46986839810853 , 83.95727803781195 ;	144.80234344193434 , 76.06812336979348 ;	83.63824888189805 , 65.3743299935233 ;	47.35379326183386 , 58.933993724246704 ;	31.260903243243753 , 47.66728455326394 ;	52.84048494641837 , 49.42519645281782 ;	68.43349356644761 , 50.29893458007114 ;	99.41423879973189 , 54.23701582416317 ;	140.43662401983312 , 57.78533468822957 ;	184.6549923917499 , 61.71392210264832 ;	207.38871018966236 , 102.29672951947487 ;	275.30206919268994 , 114.96724053260937 ;	293.2644103714822 , 185.05199500385703 ;	238.59645001787416 , 113.10569348255757 ;	210.8308477273696 , 169.92301726639607 ;	112.4366981380515 , 149.69753342557786 ;	37.934296161388836 , 82.22182428074302 ;	78.136095302918 , 56.446901114069455 ;	129.68187521876968 , 60.96949440117183 ;	263.01463136305887 , 72.05488641437643 ;
             ]'  ;  
else
    points_2D_reported = points_2d_arg;
end
if ~exist('points_3d_arg','var') 
    points_3D_reported_FLU = [ ... 
        1.7717174526377166 , -1.014855736746979 , 0.6449999999999999 ;	1.642219750065936 , -1.026272611478297 , 0.3549999999999999 ;	2.1116922597458285 , -0.9341562117494199 , 0.645 ;	1.9820979593314398 , -0.9444186334960223 , 0.355 ;	2.3898963582177686 , -0.9349478401743129 , 0.6450000000000001 ;	2.6032714267209642 , -0.9036965824601315 , 0.6450000000000001 ;	2.6628059086877585 , -0.21156018251910647 , 0.645 ;	2.743537686473329 , 0.3291713925461128 , 0.645 ;	3.1656936039137213 , 1.326330333387876 , 0.645 ;	3.1597747648024805 , 0.9422865604092621 , 0.6449999999999999 ;	3.154224126056554 , 0.6998487784652108 , 0.645 ;	3.0574971695883097 , 0.13513209977951893 , 0.645 ;	3.0375155832876106 , -0.32412310917440856 , 0.645 ;	3.028721123002628 , -0.7563925598051209 , 0.645 ;	3.0234366857776207 , -0.9625360578323743 , 0.355 ;	2.949934868393747 , -1.5002641928531644 , 0.3549999999999999 ;	2.318718481951249 , -1.8871884647593191 , 0.355 ;	2.280621502760777 , -1.633004103631637 , 0.6450000000000001 ;	2.277063121141343 , -1.5030528130637841 , 0.35500000000000004 ;	2.23069390534198 , -0.9663865655464772 , 0.355 ;	2.179028004770522 , -0.33460366612249515 , 0.645 ;	1.5163367184896286 , 0.31446490105955277 , 0.6450000000000001 ;	1.5527745728426372 , -0.037352994653233725 , 0.645 ;	1.7793715318652716 , -1.2328781175600245 , 0.645 ; 
    ]'  ;               
else
    points_3D_reported_FLU = points_3d_arg;
end

discard_bad_points = false;
% discard the last 4 points: these are BAD
if discard_bad_points 
    points_2D_reported = points_2D_reported(:,1:20);
    points_2D_reported = [ points_2D_reported(:,1:8) , points_2D_reported(:,12:20) ] ;
end;
    
if draw_axes_
    fig_h_points_2D_reported = figure2('points_2D_reported 1');
    set(gca,'Ydir','reverse')
    plot2_rows(points_2D_reported,'rx');
    for ii_ = 1 : size(points_2D_reported,2)
        text( points_2D_reported(1,ii_) , points_2D_reported(2,ii_) , sprintf('%d', ii_) );
    end       
end


% discard the last 4 points: these are BAD
if discard_bad_points
    points_3D_reported_FLU = points_3D_reported_FLU(:,1:20);
    points_3D_reported_FLU = [ points_3D_reported_FLU(:,1:8) , points_3D_reported_FLU(:,12:20)] ;
end;

points_3D_reported_world_FLU =  vertcat(points_3D_reported_FLU, ones( 1 ,  size(points_3D_reported_FLU, 2) ) ) ;

if draw_axes_
    fig_h_points_3D_reported_world_FLU = figure3('points_3D_reported_world_FLU 1');
    plot3_rows(points_3D_reported_world_FLU,'rx');
    plot3(0,0,0,'bo');  text( 0 , 0 , 0 , '\{O\}');
    for ii_ = 1 : size(points_3D_reported_world_FLU,2)
        text( points_3D_reported_world_FLU(1,ii_) , points_3D_reported_world_FLU(2,ii_) , points_3D_reported_world_FLU(3,ii_), sprintf('%d', ii_) );
    end        
    % draw_axes_direct_c(world_to_camera_good_rotm, world_to_camera_good_trans, 'world_to_camera_good', 1, 'k')
end


% num_iternations = num_iterations_;
num_points = 8;
num_points_min = 4;
split_z = true;
% draw_axes_ = draw_axes_;
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