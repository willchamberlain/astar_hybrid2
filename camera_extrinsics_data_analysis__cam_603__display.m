addpath('/mnt/nixbig/ownCloud/project_code'); 
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

%%
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


%%
points_2D_reported = [ ... 
        38.97217696070338 , 118.2824709789849 ;	32.034306321017944 , 192.466939933053 ;	93.31441891387527 , 96.54772083404143 ;	87.32955212333361 , 157.37000945678204 ;	126.46986839810853 , 83.95727803781195 ;	144.80234344193434 , 76.06812336979348 ;	83.63824888189805 , 65.3743299935233 ;	47.35379326183386 , 58.933993724246704 ;	31.260903243243753 , 47.66728455326394 ;	52.84048494641837 , 49.42519645281782 ;	68.43349356644761 , 50.29893458007114 ;	99.41423879973189 , 54.23701582416317 ;	140.43662401983312 , 57.78533468822957 ;	184.6549923917499 , 61.71392210264832 ;	207.38871018966236 , 102.29672951947487 ;	275.30206919268994 , 114.96724053260937 ;	293.2644103714822 , 185.05199500385703 ;	238.59645001787416 , 113.10569348255757 ;	210.8308477273696 , 169.92301726639607 ;	112.4366981380515 , 149.69753342557786 ;	37.934296161388836 , 82.22182428074302 ;	78.136095302918 , 56.446901114069455 ;	129.68187521876968 , 60.96949440117183 ;	263.01463136305887 , 72.05488641437643 ;
    ]  ;  
points_3D_reported_FLU = [ ... 
        1.7717174526377166 , -1.014855736746979 , 0.6449999999999999 ;	1.642219750065936 , -1.026272611478297 , 0.3549999999999999 ;	2.1116922597458285 , -0.9341562117494199 , 0.645 ;	1.9820979593314398 , -0.9444186334960223 , 0.355 ;	2.3898963582177686 , -0.9349478401743129 , 0.6450000000000001 ;	2.6032714267209642 , -0.9036965824601315 , 0.6450000000000001 ;	2.6628059086877585 , -0.21156018251910647 , 0.645 ;	2.743537686473329 , 0.3291713925461128 , 0.645 ;	3.1656936039137213 , 1.326330333387876 , 0.645 ;	3.1597747648024805 , 0.9422865604092621 , 0.6449999999999999 ;	3.154224126056554 , 0.6998487784652108 , 0.645 ;	3.0574971695883097 , 0.13513209977951893 , 0.645 ;	3.0375155832876106 , -0.32412310917440856 , 0.645 ;	3.028721123002628 , -0.7563925598051209 , 0.645 ;	3.0234366857776207 , -0.9625360578323743 , 0.355 ;	2.949934868393747 , -1.5002641928531644 , 0.3549999999999999 ;	2.318718481951249 , -1.8871884647593191 , 0.355 ;	2.280621502760777 , -1.633004103631637 , 0.6450000000000001 ;	2.277063121141343 , -1.5030528130637841 , 0.35500000000000004 ;	2.23069390534198 , -0.9663865655464772 , 0.355 ;	2.179028004770522 , -0.33460366612249515 , 0.645 ;	1.5163367184896286 , 0.31446490105955277 , 0.6450000000000001 ;	1.5527745728426372 , -0.037352994653233725 , 0.645 ;	1.7793715318652716 , -1.2328781175600245 , 0.645 ; 
    ]  ;               
size(points_2D_reported)
size(points_3D_reported_FLU)
pixels_2D_cam_603_2018_02_13    =   points_2D_reported      ;
points_3D_cam_603_2018_02_13    =   points_3D_reported_FLU      ;
size(pixels_2D_cam_603_2018_02_13)
size(points_3D_cam_603_2018_02_13)

%%
%  Load the data
camera_extrinsics_data_analysis__cam_603()


%%
% check for outliers
plot2_rows(pixels_2D_cam_603_2018_02_13')

plot3_rows(points_3D_cam_603_2018_02_13', 'rx')

 grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');


%%
 
 % one completely rubbish outlier 
 points_3D_cam_603_2018_02_13(points_3D_cam_603_2018_02_13(:,1)>15 , :)
 
 % prep to remove outlier
 pixels_2D_cam_603_2018_02_13_full  = pixels_2D_cam_603_2018_02_13  ;
 points_3D_cam_603_2018_02_13_full = points_3D_cam_603_2018_02_13  ;
 
 pixels_2D_cam_603_2018_02_13 = pixels_2D_cam_603_2018_02_13_full(points_3D_cam_603_2018_02_13_full(:,1)<=15 , :);
 points_3D_cam_603_2018_02_13 = points_3D_cam_603_2018_02_13_full(points_3D_cam_603_2018_02_13_full(:,1)<=15 , :);

% recheck for outliers
plot2_rows(pixels_2D_cam_603_2018_02_13')

plot3_rows(points_3D_cam_603_2018_02_13', 'rx')


%%

indices = randi(      size(   pixels_2D_cam_603_2018_02_13  , 1  )    , 1 , 4    )

pixels_2D_cam_603_2018_02_13(indices);
points_3D_cam_603_2018_02_13(indices);

points_3D_cam_603_2018_02_13_upper_indices =  points_3D_cam_603_2018_02_13(:,3) >0.5 ;
points_3D_cam_603_2018_02_13_lower_indices  =  points_3D_cam_603_2018_02_13(:,3) >0.5 ;

%

results = [];
% pick 100 sets 
num_iternations                             = 5000;
% pick sets of 4 points from the whole set 
num_data_points_per_sample  =  12; %12; % 4; %  8;
% use both upper and lower features 
split_z                                             = true;
num_lower = ceil(num_data_points_per_sample/2);
num_upper = num_data_points_per_sample-num_lower;
lower_indices                               = zeros(1, num_lower,'uint64');
upper_indices                               = zeros(1, num_upper,'uint64');
% record the datasets and the results
%   each iteration:  indices of the data points , residuals
datapoint_sets = zeros(num_iternations, num_data_points_per_sample, num_data_points_per_sample, 'uint64');
% don't draw: slows things down 
draw_axes_                                  = false; 
% 
tukey_step_size = 25;
tukey_num_steps = 8;
reprojected_errs_tukey = [] ;
reprojected_errs_tukey_sum = [] ;
reprojected_errs = [];
reprojected_errs_bad = [];
% reprojected_errs_tukey = zeros(num_iternations, tukey_num_steps, size(   pixels_2D_cam_603_2018_02_13  , 1  ) , 2
for ii_ = 1:num_iternations 
     try
        % pick points  -  upper
        num_picked = 0;
        while num_picked < num_upper 
            index = randi ( size(points_3D_cam_603_2018_02_13,1) );
            if points_3D_cam_603_2018_02_13_upper_indices(index)
                upper_indices(1,num_picked+1)=index;
                num_picked = num_picked+1;
            else 
                display('pick again')
            end
        end
        % pick points  -  lower
        num_picked = 0;
        while num_picked < num_lower 
            index = randi ( size(points_3D_cam_603_2018_02_13,1) );
            if points_3D_cam_603_2018_02_13_lower_indices(index)
                lower_indices(1,num_picked+1)=index;
                num_picked = num_picked+1;
            else 
                display('pick again')
            end
        end     
        %-- estimation    
        [aa , bb , cc  , world_to_camera_est_FLU, world_to_local_FLU,  est_rotm_FLU , solution_set , Alph ] = ...
            func_cam_605_select_points_input_2( camera_K , Radial , Tangental , ...
            [ pixels_2D_cam_603_2018_02_13( upper_indices ,: ) ; pixels_2D_cam_603_2018_02_13( lower_indices ,: )  ]'   , ...
            [ points_3D_cam_603_2018_02_13( upper_indices ,: ) ;   points_3D_cam_603_2018_02_13( lower_indices ,: )  ]'   , ...
            draw_axes_ );
        results(ii_).world_to_camera_est_FLU = world_to_camera_est_FLU;
        results(ii_).world_to_local_FLU = world_to_local_FLU;
        results(ii_).est_rotm_FLU = est_rotm_FLU;
        results(ii_).solution_set = solution_set;
        results(ii_).Alph = Alph;
        results(ii_).datapoint_set_indices = [ upper_indices lower_indices ];
        
        
        world_to_camera_rotm  = world_to_camera_est_FLU(1:3 , 1:3) ;     
        world_to_camera_trans = world_to_camera_est_FLU(1:3 , 4) ;        
        world_to_local_FLU_ = [ [ world_to_camera_rotm' , world_to_camera_rotm' * world_to_camera_trans .* -1]  ;  [ 0 0 0 1 ] ]  ;
        world_to_local_FLU = world_to_local_FLU_;  % !!!!!!!!

        % now calculate residuals against all the datapoints 
                %  /mnt/nixbig/ownCloud/project_code/iterate_sample_cam_605_pnp_reproject_2.m   lines 148+

        points_3D_reported_local_FLU =  world_to_local_FLU * [ points_3D_cam_603_2018_02_13 ones(size(points_3D_cam_603_2018_02_13,1),1) ]' ;  % points_3D_reported_world_FLU ;
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
        reprojected_errs( ii_ , : , : ) = abs( pixels_2D_cam_603_2018_02_13' - reprojected__(1:2,:) )  ;
        reprojected_errs_bad( ii_ , : )  =  reprojected__( 1 , : ) < 0  |  reprojected__( 2 , : ) < 0 ;        
        reprojected_errs_sq = squeeze(reprojected_errs(ii_,:,:)).^2    ;
        reprojected_errs_euc( ii_,:,:)  =  sqrt(sum(reprojected_errs_sq,1));            
        for tukey_num_ = 1:tukey_num_steps
            reprojected_errs_euc_temp = squeeze(reprojected_errs_euc( ii_,:,:));
            reprojected_errs_euc_temp( reprojected_errs_euc_temp > tukey_num_*25 ) = tukey_num_*25;
            reprojected_errs_tukey( ii_, tukey_num_ , : , : ) = reprojected_errs_euc_temp;
            reprojected_errs_tukey_sum( ii_) = sum(sum(reprojected_errs_euc_temp));
        end
     
     catch
         errors(ii_).datapoint_set_indices = [ upper_indices lower_indices ];
     end
end   

% reprojected_errs_tukey

results(1).solution_set(1).error + results(1).solution_set(2).error  + results(1).solution_set(3).error + results(1).solution_set(4).error 

residual_sum = zeros(size(results,2),1);
for ii_ = 1:size(results,2)
    fprintf('ii_=%d\n',ii_)
    residual_sum(ii_) = 0;
    for jj_ = 1 : size(results(ii_).solution_set,2)
        residual_sum(ii_) = residual_sum(ii_) + results(ii_).solution_set(jj_).error;
    end
end

residual_sum
[val,idx]=min(residual_sum)
results(idx)

t_string = datetime('now','Format','yyyy-MM-dd''T''HHmmss');

filename = sprintf('/mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/results/2018_02_13_whole_set_analyses/cam_603_2018_02_13_all_%dpt_%s.mat', ...
 num_data_points_per_sample, ...
 t_string);
save(filename,'results');

strrep(strrep( sprintf( '%s' , datetime) , ':' , '_'), ' ', '_')

%%

% look for some sort of consistency 
threshold = 20;
thresholded_inliers = reprojected_errs_euc < threshold; 
thresholded_outliers = reprojected_errs_euc >= threshold;
thresholded_inliers_summed = sum(thresholded_inliers,1)
thresholded_outliers_summed = sum(thresholded_outliers,1)

[thresholded_inliers_summed_sorted,...
    thresholded_inliers_summed_sorted_indices] = sort(thresholded_inliers_summed,'descend') 
thresholded_inliers_summed_sorted_indices(1:4)
thresholded_inliers_summed(  thresholded_inliers_summed_sorted_indices(1:4)  )
thresholded_inliers_summed_sorted_indices(1:4)

%%

squeeze(reprojected_errs(1,:,:))

for ii_ = 1:size(reprojected_errs,1)
    for jj_ = 1:size(reprojected_errs,3)
        reprojected_errs_euc(ii_,jj_) = norm(reprojected_errs( ii_ , : , jj_ ) , 2);
    end
end
find(reprojected_errs_euc>1000)
y = prctile(reshape(reprojected_errs_euc,240,1),[2.5 25 50 75 97.5])  % BAD!!  2.5%=89 25% = 240 

%%
plot(residual_sum)
residual_sum_cropped = residual_sum;
[ counts,edges ] = histcounts(residual_sum_cropped ) 
residual_sum_cropped(residual_sum_cropped > edges(2)) = -1;
[ counts,edges ] = histcounts(residual_sum_cropped ) 
Y = prctile(residual_sum, 99)

% could work down through the percentiles
y = prctile(residual_sum,[2.5 25 50 75 97.5]) % a useful summary of x

% ... look for datapoints consistently in the bad estimates ... or in the good ones ... 
find(residual_sum > 10277)

bad_estimates = results(find(residual_sum >= 10277))
for ii_ = 1:size(bad_estimates,2)
    z_bad(ii_,:) = bad_estimates(ii_).datapoint_set_indices;
end

good_estimates = results(find(residual_sum < 10277))
for ii_ = 1:size(good_estimates,2)
    z_good(ii_,:) = good_estimates(ii_).datapoint_set_indices;
end


plot3_rows(points_3D_cam_603_2018_02_13', 'go')
hold on; grid on; axis equal; 
plot3_rows(points_3D_cam_603_2018_02_13( reshape(z_bad, size(z_bad,1)*size(z_bad,2) , 1) ,:)', 'rd')
plot3_rows(points_3D_cam_603_2018_02_13( reshape(z_good, size(z_good,1)*size(z_good,2) , 1) ,:)', 'bs')
hold on; grid on; axis equal; 


% 10     8     2    24
% size(reprojected_errs_tukey)
squeeze(reprojected_errs_tukey(10,8,1,:))
for ii_ = 1:size(reprojected_errs_tukey,1)
        for jj_ = 1:size(reprojected_errs_tukey,4)
            sqrt(sum(reprojected_errs_tukey(ii_,8,:,jj_).^2))
        end
end

reprojected_errs_tukey_euc


squeeze(reprojected_errs_tukey(10,8,:,:))

plot3_rows(points_3D_reported_local_FLU, 'rx'); grid on; xlabel('x'); ylabel('y')


reprojected_errs_tukey_sum

plot(reprojected_errs_tukey_sum)

min(reprojected_errs_tukey_sum)

reshape

squeezed = squeeze(reprojected_errs_tukey(:,8,:,:))
size(squeezed)
squeezed()
plot_rows(  squeezed' )