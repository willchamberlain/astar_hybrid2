
%  Developed in   -   /mnt/nixbig/ownCloud/project_code/temp__vrep_drive_pioneer_2018_10_16.m

%   map_image_good_spots  is set up in  temp__vrep_drive_pioneer_2018_10_16.m
%- pick a spot - start 

% draw - the map
    idisp(map_image_good_spots)

bad_points = []
while true
    y = randi(size(map_image_good_spots,1))
    x = randi(size(map_image_good_spots,2))
    bad_points(:,end) = [x,y]
    if map_image_good_spots(y,x) >= 1
        FoV_centre_1 = [x;y;0]  ;
        break
    end
end

% draw - any bad points which were evaluated
    if size(bad_points,1) > 0
        hold on;    plot( bad_points(:,1) , bad_points(:,2) , 'bx' )
    end

% draw - the good point which was found    
    hold on; plot(FoV_centre_1(1),FoV_centre_1(2),'gd', 'LineWidth',2)        

%- pick a spot - end
