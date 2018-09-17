%% Visualise Field of View with uncertainty

%%
addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%
%{
    Options
        rays through each pixel , in each pose , opacity weighted by probability
        planes through each fustrum edge , in each pose on the limits of probability/confidence bound, 
%}

true_pose_rot = rotz(deg2rad(10))  * roty(deg2rad(10))  * rotz(deg2rad(10))  ;
true_pose_transl = [0 ; 0 ; 2] ;
true_pose = rt2tr( true_pose_rot  , true_pose_transl )    ;

figure_named( 'check' );  hold on; grid on ; axis equal ; xlabel('x') ; ylabel('y') ; zlabel('z') ;
draw_axes_direct(true_pose_rot, true_pose_transl, '', 2) 

%%
%{ 
could go back and estimate a set of poses from the other script, but here I can just take 
one Guassian on the orientation and another Gaussian on the position.  Start simple: make 
four scenarios, combinations of big/small variance for each. Then add another with non-zero 
mean and check how it looks.
Note: want to abstract and record the distribution, and the specific samples, so that I can 
save and reload it to recreate each trial.


LATER Alternative - sample position as Gaussian, sample target from a small Gaussian on the 
floor plane - might look more like the estimated poses..
%}

num_samples = 1000  ;

% x,y,z 
num_dim = 3  ;
pose_distribution_position = normrnd(0,  0.05, num_dim , num_samples)  ;
% hist(pose_distribution_position,100)

% pitch, yaw
num_dim = 2  ;
pose_distribution_orientation_py = normrnd(0, 0.05  ,num_dim*num_samples,1)  ;
% hist(pose_distribution_orientation_py,100)

% roll
num_dim = 1  ;
pose_distribution_orientation_r = normrnd(0, 0.01  ,num_dim*num_samples,1)  ;
% hist(pose_distribution_orientation_r,100)

rpy_set =  rpy2r( ...
    pose_distribution_orientation_r, ...
    pose_distribution_orientation_py(1:num_samples), ...
    pose_distribution_orientation_py(num_samples+1:num_samples*2)  );
    size(  pose_distribution_orientation_py(1:round(num_samples))  )
    size(  pose_distribution_orientation_py(round(num_samples)+1:num_samples*2)  )

    size(rpy_set)  % 3 3 1000
    
    size(pose_distribution_position) % 3 1000 
    pose_distribution_position(1)'
    
    tr_set = rt2tr(rpy_set,pose_distribution_position)  ;
    
    
    noisy_pose_subset_size = 100 
    %       noisy_pose_subset(:,:,1:noisy_pose_subset_size)   = tr_set(:,:,1:noisy_pose_subset_size)  ;    
    noisy_pose_subset = tr_set(:,:,1:noisy_pose_subset_size)  ;    
    
    figure_named(sprintf('true_pose plus %i estimates',noisy_pose_subset_size )) ;    hold on; grid on ; axis equal ; xlabel('x') ; ylabel('y') ; zlabel('z') ;
    draw_axes_direct(true_pose_rot, true_pose_transl, '', 2) 
    for ii_ = 1:noisy_pose_subset_size
        tr_ = noisy_pose_subset(:,:,ii_) * true_pose        ;
        draw_axes_direct(tr_(1:3,1:3), tr_(1:3,4), '', 2 )      ;
    end
    
    % draw fustrum - assume a 4:3 aspect ratio, 60 degrees wide --> high 45 degrees ;
    distance_to_target = 5 ;
    vec_width_left = tan(deg2rad(60/2)) * distance_to_target  ;
    vec_height_up = tan(deg2rad(45/2)) * distance_to_target  ;
    camera_optical_centre = [ 0 ; 0 ; 0 ] ;
    fustrum_corners = [ ...
        distance_to_target -vec_width_left ,  -vec_height_up 
        distance_to_target -vec_width_left ,   vec_height_up 
        distance_to_target  vec_width_left ,   vec_height_up 
        distance_to_target  vec_width_left ,  -vec_height_up 
        ]'  ;
    figure  ;  plot2_rows( fustrum_corners(2:3,:) )  ;
    figure_named('fustrum') ;    hold on; grid on ; axis equal ; xlabel('x') ; ylabel('y') ; zlabel('z') ;   
    for ii_ = 1:size(fustrum_corners,2)-1
        plot3_rows( [ fustrum_corners(:,ii_) fustrum_corners(:,ii_+1) ] )  ;
    end
    plot3_rows( [ fustrum_corners(:,size(fustrum_corners,2)) fustrum_corners(:,1) ] )  ;
    for ii_ = 1:size(fustrum_corners,2)
        plot3_rows( [ fustrum_corners(:,ii_) camera_optical_centre ] )  ;
    end
    
draw_cam_fustrum_3d( fustrum_corners , camera_optical_centre , 'm'  )
    
    fustrum_corners_noisy_hom_total = []  ;
    for ii_ = 1:noisy_pose_subset_size
        fustrum_corners_noisy_hom = noisy_pose_subset(:,:,ii_) *   euc2hom(fustrum_corners) ;   
        fustrum_corners_noisy_hom_total = [ fustrum_corners_noisy_hom_total fustrum_corners_noisy_hom]  ;
        camera_optical_centre_noisy = noisy_pose_subset(1:3,4,ii_)  ;
        fustrum_corners_noisy_hom_total = [ fustrum_corners_noisy_hom_total euc2hom(camera_optical_centre_noisy)]  ;
        draw_cam_fustrum_3d( h2e(fustrum_corners_noisy_hom) , camera_optical_centre_noisy , 'r'  )
    end
    
    point_set = [  h2e(fustrum_corners_noisy_hom_total)'  ; camera_optical_centre']  ;
    point_set_hull = convhulln( point_set )  ;
    
    for ii_ = 1:size(point_set_hull,1)  
        points_of_patch = ( point_set(point_set_hull( ii_ , :),:) )  ;
        patch(points_of_patch(:,1),points_of_patch(:,2),points_of_patch(:,3), 'm', 'FaceAlpha', 0.25, 'EdgeColor','m', 'EdgeAlpha', 0.25+0.1)  ;
    end
    
    

    %{
    LATER Alternative - sample position as Gaussian, sample target from a small Gaussian on the 
    floor plane - might look more like the estimated poses..
    %}
    
    %%
    %  Sampling is messy - appearance of rays
    
    % Convolve fustrum with a Gaussian ?  Convolve(?) fustrum corner rays with the orientation 
    %Gaussian 
    % Multiply out the Gaussians ?
    %  Try, then ask once have some example images.
    %
    %  Like a guided filter ?? 
    
    % try to find product of camera's  pose xy (parallel to image sensor) uncertainty with the camera's orientation xy uncertainty 
    % - would expect to be able to approximate it at the optical axis as a truncated cone, then extend that to the fustrum limits
    
    gaussian_kernel_small =  kgauss(5,140)   ;
    gaussian_kernel =  kgauss(50,140)   ;
    idisp( gaussian_kernel )  
    hist(gaussian_kernel)
    surf(gaussian_kernel)
    
    max(max(gaussian_kernel))
    idisp(   gaussian_kernel > 0.1*max(max(gaussian_kernel))     )
    idisp(   gaussian_kernel > 0.95*max(max(gaussian_kernel))     )
    
    figure;   surf(gaussian_kernel)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;
    figure;   surf(gaussian_kernel*gaussian_kernel_small)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;
    figure;   surf(gaussian_kernel_small*gaussian_kernel)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;
    figure;   surf(gaussian_kernel_small*gaussian_kernel*gaussian_kernel_small)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;
    figure;   surf(gaussian_kernel*gaussian_kernel_small*gaussian_kernel)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;

    figure; idisp(conv2(gaussian_kernel,gaussian_kernel_small,'same') )  ;
    figure; surf(conv2(gaussian_kernel,gaussian_kernel_small,'same') )  ;
    figure_named('diff'); surf(   conv2(gaussian_kernel,gaussian_kernel_small,'same') - gaussian_kernel  )  ;
    figure_named('diff'); surf(   gaussian_kernel - conv2(gaussian_kernel,gaussian_kernel_small,'same')   )  ;
    
    
    figure_named('diff_mult');   surf(gaussian_kernel - gaussian_kernel*gaussian_kernel_small*gaussian_kernel)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;
    
     figure; idisp(gaussian_kernel)
    
     
    
    gaussian_kernel_joint =  kgauss(   (50^2 * 5^2) / ( 50^2 + 5^2)  ,140)   ;
    figure_named('joint sigma'); surf(gaussian_kernel_joint )  ;
    figure_named('diff_joint');   surf(gaussian_kernel - gaussian_kernel_joint)  ;  xlabel('x') ; ylabel('y') ; zlabel('z') ;
    
    % ???????
    
    %%
    
    
    
