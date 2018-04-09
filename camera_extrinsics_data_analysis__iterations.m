
addpath('/mnt/nixbig/ownCloud/project_code'); 
addpath('/mnt/nixbig/ownCloud/project_AA1__1_1/code/mutliview_pose_estimation/src/main/matlab/camera_pose_from_known_3D_point_features_PnP/')

%%


points_2D_reported = [ ... 
	38.97217696070338 , 118.2824709789849 ;	32.034306321017944 , 192.466939933053 ;	93.31441891387527 , 96.54772083404143 ;	87.32955212333361 , 157.37000945678204 ;	126.46986839810853 , 83.95727803781195 ;	144.80234344193434 , 76.06812336979348 ;	83.63824888189805 , 65.3743299935233 ;	47.35379326183386 , 58.933993724246704 ;	31.260903243243753 , 47.66728455326394 ;	52.84048494641837 , 49.42519645281782 ;	68.43349356644761 , 50.29893458007114 ;	99.41423879973189 , 54.23701582416317 ;	140.43662401983312 , 57.78533468822957 ;	184.6549923917499 , 61.71392210264832 ;	207.38871018966236 , 102.29672951947487 ;	275.30206919268994 , 114.96724053260937 ;	293.2644103714822 , 185.05199500385703 ;	238.59645001787416 , 113.10569348255757 ;	210.8308477273696 , 169.92301726639607 ;	112.4366981380515 , 149.69753342557786 ;	37.934296161388836 , 82.22182428074302 ;	78.136095302918 , 56.446901114069455 ;	129.68187521876968 , 60.96949440117183 ;	263.01463136305887 , 72.05488641437643 ;
         ]'  ;  
points_3D_reported_FLU = [ ... 
	1.7717174526377166 , -1.014855736746979 , 0.6449999999999999 ;	1.642219750065936 , -1.026272611478297 , 0.3549999999999999 ;	2.1116922597458285 , -0.9341562117494199 , 0.645 ;	1.9820979593314398 , -0.9444186334960223 , 0.355 ;	2.3898963582177686 , -0.9349478401743129 , 0.6450000000000001 ;	2.6032714267209642 , -0.9036965824601315 , 0.6450000000000001 ;	2.6628059086877585 , -0.21156018251910647 , 0.645 ;	2.743537686473329 , 0.3291713925461128 , 0.645 ;	3.1656936039137213 , 1.326330333387876 , 0.645 ;	3.1597747648024805 , 0.9422865604092621 , 0.6449999999999999 ;	3.154224126056554 , 0.6998487784652108 , 0.645 ;	3.0574971695883097 , 0.13513209977951893 , 0.645 ;	3.0375155832876106 , -0.32412310917440856 , 0.645 ;	3.028721123002628 , -0.7563925598051209 , 0.645 ;	3.0234366857776207 , -0.9625360578323743 , 0.355 ;	2.949934868393747 , -1.5002641928531644 , 0.3549999999999999 ;	2.318718481951249 , -1.8871884647593191 , 0.355 ;	2.280621502760777 , -1.633004103631637 , 0.6450000000000001 ;	2.277063121141343 , -1.5030528130637841 , 0.35500000000000004 ;	2.23069390534198 , -0.9663865655464772 , 0.355 ;	2.179028004770522 , -0.33460366612249515 , 0.645 ;	1.5163367184896286 , 0.31446490105955277 , 0.6450000000000001 ;	1.5527745728426372 , -0.037352994653233725 , 0.645 ;	1.7793715318652716 , -1.2328781175600245 , 0.645 ; 
]'  ;            

points_3D_reported_world_FLU =  vertcat(points_3D_reported_FLU, ones( 1 ,  size(points_3D_reported_FLU, 2) ) ) ;

%%


for ii_ = 4:12
    for jj_ = 1:4        
        tic
        fprintf('\nsample size %d iteration %d start\n', ii_,jj_)
        [ world_to_camera_rotm_hist , world_to_camera_trans_hist , best_err_tukey_100_100_idx , best_err_tukey_50_50_idx ] ...
            = iterate_sample_cam_605_pnp_reproject_2(100, ii_, 0);
        world_to_camera_rotm_hist_exp(:,:,:,ii_,jj_)=world_to_camera_rotm_hist;
        world_to_camera_trans_hist_exp(:,:,:,ii_,jj_)=world_to_camera_trans_hist;
        best_err_tukey_100_100_idx_exp(ii_,jj_)=best_err_tukey_100_100_idx;
        best_err_tukey_50_50_idx_exp(ii_,jj_)=best_err_tukey_50_50_idx;
        drawnow
        fprintf('\nsample size %d iteration %d end\n', ii_,jj_)
        toc
    end
end


%%

% 100     3     3    12     4  =  
size(world_to_camera_rotm_hist_exp)     %   100x3x3 : 100 iterations, 6 DOF pose-   12 x 4 : 4-12 for sample size, 4 iterations for each sample size 
size(best_err_tukey_100_100_idx_exp)    % 12 x 4 : 4-12 for sample size, 4 iterations for each sample size 


sample_size_min=11;
sample_size_max=12;
sample_sizes = [4 8 12]
figure('Name',sprintf('sample sizes %d to %d ', sample_size_min, sample_size_max));  hold on;  xlabel('x'); ylabel('y'); zlabel('z');
for ii_ = sample_sizes
    color_ = [  ii_*0.1 , 0.5 , 1- (ii_*0.1)  ];
    for jj_ = 1:4 
        draw_axes_direct( ...
            squeeze(world_to_camera_rotm_hist_exp(  best_err_tukey_100_100_idx_exp(ii_,jj_) ,:,:,ii_ , jj_)) , ...
            squeeze(world_to_camera_trans_hist_exp( best_err_tukey_100_100_idx_exp(ii_,jj_) ,:,:,ii_ , jj_))' , ...
            sprintf('%d\\_%d',ii_,jj_) , ...
            0.2); % , ...
            %color_);
    end
end
plot3_rows(points_3D_reported_world_FLU,'rx');
plot3(0,0,0,'bo');  text( 0 , 0 , 0 , '\{O\}');
axis equal; grid on;
xlim([-1,3.5])