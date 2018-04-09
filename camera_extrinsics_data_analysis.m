%{
source
    2018_02_12(ish)
    Mobile Lab
    4 cameras
    1 Pioneer   w. fiducials   w. visual model
    per-run
        

data
    observation file per-camera per-run
    image files per-camera per-run

objectives
    see PC/Will supervisory meeting 
    identify runs which the robot has bad localisation for
    

outcomes
    
    thesis chapter
    journal paper section

steps
    copy datapoints variables out of the observation data files
        points_2D
        points_3D
        filename
    paste into this script
    click-through GUI to change of direction
    run asfddsf.m(global start_idx , global current_idx )  to
        copy the data to variables and 
        save them in 
            format that can be loaded , and read as plain text 
            (? table ?)
            file with name 
                [filename]__[start_idx]__[current_idx]__points_[2|3]D.txt
        LATER : create script to copy image files
                    nice-to-have
                    images don't align with observations - can have 1 or more observations per image
                    find matching images for any interesting data later 
        -->  mix-and-match data files
    load data file(s) 
        estimate pose from them 
            analysis of bulk statistics 
                is normally-distributed 
                    how check in 3D - can check each dimension separately to a 2D histogram
                    null hypothesis --> Bayesian --> how likely is it that has distribution --> Bayesian estimation video on Youtube 


        analysis of physical spread of data     -       PCA ?
        analysis of image spread of data        -       PCA ?
    


%}

[coeff, score, latent, tsquared, explained, mu] = pca(x,varargin)

%%
addpath('/mnt/nixbig/ownCloud/project_code/')

global pixels_2D
global points_3D

% observation data files are in 
%       as recorded : per-cam-per-run data and images
%           /mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/results/2018_02_13/cam_xxx/VOS_data_yyyy_MM_dd_HH24_MM_ss
%       as recorded but renamed and copied 
%           /mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/results/2018_02_13_consolidated
% output split files are in 
%       /mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/results/  
% 
% 2018_02_13_20_56_29 : 
pixels_2D=[...
	310.466677119259 , 260.11320254658796 ;	289.6051947754041 , 204.40428273931408 ;	266.6345046051176 , 234.33709264036438 ;	109.88189059845593 , 106.5193753285479 ;	80.06695832395782 , 91.96185831390666 ;	44.92241545687989 , 124.5869466645928 ;	111.02821365261457 , 155.29939844294032 ;	184.7395522261935 , 156.87562664572627 ;	171.58380747501312 , 190.16982140069751 ;	307.14336344089094 , 158.84801705747785 ;	283.7461898258991 , 192.1151965565718 ; ]; 
points_3D = [ ...
	2.59479349847569 , 0.4357599711352126 , 0.355 ;	2.5501216787596457 , 0.6254166295396878 , 0.645 ;	2.5592020842464587 , 0.7550991119195183 , 0.355 ;	2.786075844683932 , 2.5307064752530657 , 0.6450000000000001 ;	2.8357618088955157 , 3.198244220399884 , 0.645 ;	2.113620636822215 , 2.848007105363655 , 0.645 ;	2.2469313879895325 , 1.9343416288126971 , 0.645 ;	2.5611398697797214 , 1.5261145964697502 , 0.645 ;	2.4816569932741306 , 1.628985740831991 , 0.355 ;	3.013423148988654 , 0.9446025121488514 , 0.645 ;	3.001754123200808 , 1.0740777369494385 , 0.355 ; ]; 
% 2018_02_13_20_56_29 : 
pixels_2D=[...
	298.18489365461784 , 129.85240864202956 ;	168.86311157504372 , 104.49139614435458 ;	49.77289887448287 , 146.6971998939965 ;	36.40933372982246 , 185.33581488148812 ;	41.9053560961004 , 235.7154912354582 ;	91.84724563872776 , 64.73570271526529 ;	138.2855561255116 , 47.94355430451613 ;	210.17135728665605 , 65.20212303587668 ;	191.26993843687646 , 97.50276543911954 ; ]; 
% 2018_02_13_20_56_29 : 
 points_3D = [...  
	1.3223518914373475 , -4.963947835735521 , 0.355 ;	1.7422074494144237 , -4.361273922917494 , 0.6449999999999999 ;	2.1746089688282635 , -3.782728764283933 , 0.3549999999999999 ;	1.79368561978843 , -3.6936473912114702 , 0.645 ;	1.7494360405848197 , -3.5714100207378565 , 0.355 ;	2.4615346412512453 , -4.297166345978645 , 0.645 ;	2.305812555726503 , -4.64018538361421 , 0.645 ;	1.872704925029623 , -4.885751934926204 , 0.645 ;	1.9883212046092673 , -4.826313934487414 , 0.355 ; ]; 
% do this with subplots 
global fig_handle
fig_handle = figure; 
subplot(2,1,1); 
hold on;
plot3_rows(points_3D', 'r');
% draw a line to the ground plane
for ii_ = 1: size(points_3D,1)
     plot3( [points_3D(ii_,1), points_3D(ii_,1)], [points_3D(ii_,2), points_3D(ii_,2)] , [points_3D(ii_,3), 0] ,  'Color', '[ 0.5 0.5 0.5 ]' )
end
hold on; axis equal; grid on; xlabel('x');ylabel('y');zlabel('z');

subplot(2,1,2); 
plot(pixels_2D(:,1)',max(pixels_2D(:,2))-pixels_2D(:,2)','b')
hold on; axis equal; grid on

global start_idx
global current_idx
start_idx = 1;
current_idx = start_idx;

set(fig_handle,'KeyPressFcn',@camera_extrinsics_data_analysis_keyboard_callback);

% for current_idx = start_idx:size(points_3D,1)    
%     subplot(2,1,2); hold on;
%     plot(pixels_2D(current_idx,1)',max(pixels_2D(:,2))-pixels_2D(current_idx,2)','bx')
%     
%     subplot(2,1,1); hold on;
%     plot3_rows(points_3D(current_idx,:)', 'rd');
%     pause
% end

%  copy the data out , so that can load back into points_2D_reported and points_3D_reported_FLU in /mnt/nixbig/ownCloud/project_code/iterate_sample_cam_605_pnp_reproject_2.m

%%

%%
% EXAMPLE - Test PCA on the original 3D data 
% http://nghiaho.com/?page_id=1030 

% shift to zero-mean
points_3D_zero_meaned = points_3D - repmat( mean(points_3D) , length(points_3D) , 1 ) 

% find the covariance
N = size(points_3D,1);
points_3D_covar = points_3D'*points_3D ./ N

% find the rotationpoints_3D
[U S V] = svd(points_3D_covar)
diag(S)/sum(diag(S))

% apply the rotation
points_3D_rotated = points_3D_zero_meaned * V

% reduce the dimensions 
points_3D_reduced_data = points_3D_rotated(:, 1:2); % keep first 2 components

plot2_rows(points_3D_reduced_data','r')

%%
% EXAMPLE - Test PCA on the original 3d AND 2D data 
pixels_2D_normalised = pixels_2D - repmat(  mean(pixels_2D) , length(pixels_2D) , 1 )
points_5D = [ pixels_2D_normalised points_3D  ]

points_5D_zero_meaned = points_5D - repmat( mean(points_5D) , length(points_5D) , 1 ) 

% find the covariance
N = size(points_5D,1);
points_5D_covar = points_5D'*points_5D ./ N

% find the rotation
[U S V] = svd(points_5D_covar)
diag(S)/sum(diag(S))

% apply the rotation
points_5D_rotated = points_5D_zero_meaned * V

% reduce the dimensions 
points_5D_reduced_data_2 = points_5D_rotated(:, 1:2); % keep first 2 components
plot2_rows(points_5D_reduced_data_2','r')
points_5D_reduced_data_3 = points_5D_rotated(:, 1:3); % keep first 2 components
plot3_rows(points_5D_reduced_data_3','r')  ;  axis equal; grid on; 










