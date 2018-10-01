addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%

info_about_drivers = imaqhwinfo

info_for_driver__linuxvideo = imaqhwinfo('linuxvideo')   
info_for_driver__linuxvideo = imaqhwinfo('linuxvideo',1)
info_for_driver__linuxvideo = imaqhwinfo('linuxvideo',2)   

%       imaq.reset
% webcam_object = imaq.VideoDevice('linuxvideo',1,'BGR24_1920×1080')
webcam_object = imaq.VideoDevice('linuxvideo',1)
webcam_object.release
webcam_object.DeviceProperties.WhiteBalanceTemperature = 5300
[frame metadata] = step(webcam_object);
imshow(frame);
% webcam_object.DeviceProperties

frame_hsv = rgb2hsv(frame) ;
[mask, h_range, s_range ] =temp__image_statistics_histogram_of_path( frame_hsv, 1004,1024 , 1230,964,  1500) ;
imshow(mask & frame_hsv(:,:,3)>0.7) ;
mask = mask & frame_hsv(:,:,3)>0.7  ;
    % histogram(squeeze(frame(:,:,3)) )
    % imshow(mask & frame(:,:,3)>0.6) ;
    % imshow(mask & frame(:,:,2)>0.6) ;
    % imshow(mask & frame(:,:,1)>0.6) ;

simplified = (  idilate(  ierode(mask, ones(3), 1, 'none'), ones(4), 5, 'none'));
clf; imshow(simplified); hold on;


outstats = regionprops(logical(simplified), 'BoundingBox','Centroid','Area')
[a,b]=histcounts([outstats.Area])
max([outstats.Area])

% outstats.Area > 3000

max_y = 0;
min_y_to_be_patrolbot = 8.325000000000000e+02 * 0.9
for ii_ = 1:size(outstats,1)
    if outstats(ii_).Area > 3000
        plot2_rows(outstats(ii_).Centroid','rx', 'LineWidth', 10);
        bb = outstats(ii_).BoundingBox;
%         patch([bb(1),bb(1),bb(3),bb(3)],[bb(2),bb(4),bb(4),bb(2)],'g')
%         patch([bb(1),bb(2),bb(2),bb(1)],[bb(3),bb(3),bb(4),bb(4)],'g')
%         patch([bb(3),bb(3),bb(4),bb(4)],[bb(1),bb(2),bb(2),bb(1)],'g')
%         patch([bb(2),bb(2),bb(4),bb(4)],[bb(1),bb(3),bb(3),bb(1)],'g')
        plot2_rows(bb(1:2)','bx')
        plot2_rows(bb(3:4)'+bb(1:2)','gx')  % - don't ask:  it's the bounding box as [ lower-left , offset to upper-right ]
        plot2_rows([outstats(ii_).Centroid(1);bb(4)+bb(2)],'mx');        %  <----   THIS IS THE 'BOTTOM' of the region --> homography --> edge of robot + radius in camera direction = robot location 
        
        if bb(4)+bb(2) > max_y
            max_y = bb(4)+bb(2)
        end
        
        if bb(4)+bb(2) > min_y_to_be_patrolbot 
                plot2_rows([outstats(ii_).Centroid(1);bb(4)+bb(2)],'mx', 'LineWidth',10);        %  <----   THIS IS THE 'BOTTOM' of the region --> homography --> edge of robot + radius in camera direction = robot location 
            plot2_rows([outstats(ii_).Centroid(1);bb(4)+bb(2)],'cx', 'LineWidth',2);        %  <----   THIS IS THE 'BOTTOM' of the region --> homography --> edge of robot + radius in camera direction = robot location         
        end
    end
end

%%

fig_colour_h = figure_named('raw') ;
fig_mask_h = figure_named('masked') ;

%%

%   get homography from   /mnt/nixbig/ownCloud/project_code/temp__homography_S11_20180927.m

while 1
    robot_found = false;
    robot_nearest_pixel = [-1;-1]  ;
    
    image_scaling_factor = 0.5 ;
    [full_frame metadata] = step(webcam_object);
    frame = iscale(full_frame,image_scaling_factor) ;
    display('frame')
%     frame = full_frame  ;
    figure(fig_colour_h); clf; imshow(iscale(frame,0.5));    
    figure(fig_mask_h);
    %  imshow(downsample(frame))  %  slow
    
    frame_hsv = rgb2hsv(frame)  ;
    hsv_image_ = frame_hsv ;
    h = hsv_image_(:,:,1);
    s = hsv_image_(:,:,2);
    %   mask = h>=min(h_set) & h<=max(h_set)  &  s>=min(s_set) & s<=max(s_set)   ;
    mask = h>=h_range(1) & h<=h_range(2)  &  s>=s_range(1) & s<=s_range(2)   ;    
    mask = mask & frame_hsv(:,:,3)>0.7  ;
    
    simplified = (  idilate(  ierode(mask, ones(3), 1, 'none'), ones(4), 5, 'none'))  ;
    figure(fig_mask_h); clf; imshow(simplified); hold on;    
    
    outstats = regionprops(logical(simplified), 'BoundingBox','Centroid','Area')  ;
    [a,b]=histcounts([outstats.Area]) ;
    
    max_y = 0;
    min_y_to_be_patrolbot = 8.325000000000000e+02 * 0.9 * image_scaling_factor  ;
    min_y = min_y_to_be_patrolbot ;
    for ii_ = 1:size(outstats,1)
        if outstats(ii_).Area > 3000*image_scaling_factor
            plot2_rows(outstats(ii_).Centroid','rx', 'LineWidth', 10);
            bb = outstats(ii_).BoundingBox;
            plot2_rows(bb(1:2)','bx')  ;
            plot2_rows(bb(3:4)'+bb(1:2)','gx') ; % - don't ask:  it's the bounding box as [ lower-left , offset to upper-right ]
            plot2_rows([outstats(ii_).Centroid(1);bb(4)+bb(2)],'mx');        %  <----   THIS IS THE 'BOTTOM' of the region --> homography --> edge of robot + radius in camera direction = robot location 

            if bb(4)+bb(2) > max_y
                max_y = bb(4)+bb(2)  ;
            end

            blob_y = bb(4)+bb(2)  ;
            if  blob_y > min_y_to_be_patrolbot 
                if blob_y > min_y
                    min_y = blob_y  
                    robot_found = true;
                    robot_nearest_pixel = [ outstats(ii_).Centroid(1) ; bb(4)+bb(2) ] ;   
                else
                    display(blob_y)
                end
            end            
        end
    end
    if robot_found
        plot2_rows(robot_nearest_pixel,'mx', 'LineWidth',10);        %  <----   THIS IS THE 'BOTTOM' of the region --> homography --> edge of robot + radius in camera direction = robot location 
        plot2_rows([outstats(ii_).Centroid(1);bb(4)+bb(2)],'cx', 'LineWidth',2);        %  <----   THIS IS THE 'BOTTOM' of the region --> homography --> edge of robot + radius in camera direction = robot location         
        
        maybe_pos2 = homography_Matrix_im_to_world * [robot_nearest_pixel;1]  ;
        maybe_pos2 = maybe_pos2.*(1/maybe_pos2(3))
    end
end




