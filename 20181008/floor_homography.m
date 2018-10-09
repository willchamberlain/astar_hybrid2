floor_markers_cm = [  ... % left to right as I look at them
    0,-350 ; 0,-312 ; 0,-112 ; 0,0 ;
    -100,-350 ; -100,-312 ; -100,-112 ; -100,0 ;
    -200,-350 ; -200,-312 ; -200,-112 ; -200,0 ;
    ]


im_ = imread('/mnt/nixbig/ownCloud/project_code/20181008/calibrate_phone_extrinsics/611_tripod1_1150.jpg')  ;

idisp(im_)

im_size = size(im_)

floor_markers_cm = [  ... % left to right as I look at them 
    0,-350 ; 0,-312 ; 0,-112 ; 0,0 ;
    -100,-350 ; -100,-312 ; -100,-112 ; -100,0 ;
    -200,-350 ; -200,-312 ; -200,-112 ;  -200,0;
    ] ;

%%

floor_markers_viewed_cm = [  ... % left to right as I look at them 
    0,-350 ; 0,-312 ; 0,-112 ; 0,0 ;
    -100,-350 ; -100,-312 ; -100,-112 ; -100,0 ;
    -200,-350 ; -200,-312 ; -200,-112 ;   -200,0;
    ] ;

pixels = [ ...  % for an 1280x720  %  these are (0,0) top left , (1280,720) at bottom right
438,206 ; 481,219 ;  775,307; 1029, 380 ;
275,236 ; 319,251 ; 613,365; 879,473 ;
67,274 ; 101,295;  378,451 ; 651,618 ; 
]  ;
pixels_scaled =  [ pixels(:,1)/2.0,  pixels(:,2)/1.5 ]'


[1280,720] ./ [640,480]  = [2.0,1.5]

[640,480].*2.0  = 1280x960
[640,480].*1.5  = 960x720


%  CHECK !  
%  see /mnt/nixbig/ownCloud/project_code/temp__homography_S11_20180927.m
figure; hold on; grid on; plot2_rows(pixels_scaled,'bx'); axis equal 
fig_h = gcf  ;
axis_h = gca  ;
axis_h.YDir =  'reverse' ;

for ii_ = 1:size(pixels_scaled,2)
    plot2_rows(pixels_scaled(:,ii_),'rx')  ;
    text(pixels_scaled(1,ii_),pixels_scaled(2,ii_),int2str(ii_))  ;
end
    % good: pixels look legitimate 
PIN_pixels =  pixels_scaled;   
POUT_world =  floor_markers_viewed_cm';

homography_Matrix_im_to_world = homography(PIN_pixels, POUT_world);      
%maximum residual 1.682

 [H,in] = ransac(@homography, [PIN_pixels; POUT_world], 10)
 in =    1     2     3     5     6     7     8     9    10    12

homography_Matrix_im_to_world
H
homography_Matrix_im_to_world - H

(homography_Matrix_im_to_world - H) ./ homography_Matrix_im_to_world  %  parts in a thousand difference 




