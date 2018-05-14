cam_posn = [0 0 2]  ;
cam_opticalaxis_floor_intercept = [5 0 0]  ;  %  5m
cam_opticalaxis_floor_intercept_hyp = norm(cam_opticalaxis_floor_intercept - cam_posn, 2) ;
cam_opticalaxis_floor_intercept_angle = atan ( (cam_posn(3)-cam_opticalaxis_floor_intercept(3))/(cam_posn(1)-cam_opticalaxis_floor_intercept(1)) )  ;
cam_opticalaxis_angle_to_vertical = atan(   (cam_posn(1)-cam_opticalaxis_floor_intercept(1)) / (cam_posn(3)-cam_opticalaxis_floor_intercept(3))   )  ;

% approximate pixel angle from optical axis to bottom of image - _should_ calculate 
%   this as the angle from optic through each pixel centre
angle_approx=degtorad(1:45)  ; 
for ii_ = size(angle_approx,2):-1:2
    display(ii_)
    display(angle_approx(ii_)-angle_approx(ii_-1))
    % cosine rule to get horizontal distance.
    cam_opticalaxis_floor_intercept_hyp
end


adj = cam_posn-cam_opticalaxis_floor_intercept ;
adj = adj(3) 
for ii_ = 0:0.1:cam_opticalaxis_angle_to_vertical*-1                %  can go by angle with image plane per pixel 
    opp = adj * tan(cam_opticalaxis_angle_to_vertical-ii_)
end

% CHECK the camera specs
%  https://www.devicespecifications.com/en/model/617b3011 
w=4.69 * 10^-3
h=3.52 * 10^-3  
pixel = 1.127 * 10^-6
res_w= w/pixel
res_h= h/pixel
pixel_count=res_h*res_w
focal_length = 3.8*10^-3

fov_vertical=degtorad(90)  % WRONG - measure!
angle_per_pixel = fov_vertical/res_h  % WRONG - varies
radtodeg(angle_per_pixel)  %  0.0288 degrees at full resolution 

angle_per_pixel_at_640x320 = fov_vertical/320 
radtodeg(angle_per_pixel_at_640x320)  % 0.2812 

adj = cam_posn-cam_opticalaxis_floor_intercept ;
adj = adj(3) 
angles = cam_opticalaxis_angle_to_vertical*-1:angle_per_pixel_at_640x320*-1:(cam_opticalaxis_angle_to_vertical*-1)-degtorad(45)
opps = tan(angles).*adj
opps_diffs = opps(1:end-1) - opps(2:end)
figure('Name','opps_diffs');plot(flip(opps_diffs,2)); ylim([0 0.2]); hold on; xlabel('height in image (pixels)'); ylabel('span of one pixel on floor (m)'); 

adj = 2-0.645   % at robot height
% full span of field of view
angles2 = (cam_opticalaxis_angle_to_vertical*-1)+degtorad(45):angle_per_pixel_at_640x320*-1:(cam_opticalaxis_angle_to_vertical*-1)-degtorad(45)
opps2 = tan(angles2).*adj
opps2_positive = opps2(opps2>=0)
opps_diffs2 = opps2_positive(1:end-1) - opps2_positive(2:end)
% up to 0.2m/pixel : goes to infinite at parallel to floor 
figure('Name','opps_diffs2');plot(flip(opps_diffs2,2)); ylim([0 0.2]); hold on; plot([160 160],[0 0.2]);   xlabel('height in image (pixels)'); ylabel('span of one pixel on floor (m)');

%%  Wrong: use the above - check the above later by setting the camera parallel to the floor and making the field of view 180 degrees
% try for the correct angle per-pixel from a flat image sensor 
% angles between pixel centres is difference each pixel error makes 
% first get the angles across the image
pixel_height = 1.127 * 10^-6
res_h_used = 320
pixel_centres_v =( [1:res_h_used] - res_h_used/2) * (pixel_height*0.5)
adj_focal=focal_length        
opps_pixels = pixel_centres_v(1:end-1)-pixel_centres_v(2:end)    % 
for ii_=160:320
    opps_pixels(ii_) = sum(abs(pixel_centres_v(160:ii_)))
end
for ii_=1:160
    opps_pixels(ii_) = sum(abs(pixel_centres_v(ii_:160)))
end
angles = atan(opps_pixels/adj_focal)        
radtodeg(angles(1:10))
radtodeg(angles(159:170))
radtodeg(angles(300:320))

adj = cam_posn-cam_opticalaxis_floor_intercept ;
adj = adj(3) 
opps = tan(angles).*adj
opps_diffs = opps(1:end-1) - opps(2:end)
figure('Name','opps_diffs');plot(flip(opps_diffs,2)); hold on; xlabel('height in image (pixels)'); ylabel('span of one pixel on floor (m)'); 
ylim([0 0.2]); 

%%


