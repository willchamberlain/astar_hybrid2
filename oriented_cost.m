function map__ = oriented_cost(map_in_ , camera_loc_)

map_ = zeros(size(map_in_));
map_2 = zeros(size(map_));
map_3 = zeros(size(map_));

%camera_loc_ = [40;20] ;
%map_(camera_loc_(1),camera_loc_(2)) = 1;

%idisp(map_)

%figure; surf(map_); hold on; 
%plot3(camera_loc_(1),camera_loc_(2),max(max(map_)), 'rx')

c_row = camera_loc_(1) ;
c_col = camera_loc_(2) ;
for row_ = 1:size(map_,1)
    for col_ = 1:size(map_,2)
        xy = [ col_ ; row_  ] ;
        vec = xy - [ c_col; c_row ]  ;
        angle_to_x_axis = atan2(vec(2),vec(1)) ;
        map_(row_,col_)= angle_to_x_axis ;
        map_2(row_,col_)= angle_to_x_axis + pi/2;
    end
end
% figure; surf(map_); hold on; 
% plot3(camera_loc_(1),camera_loc_(2),max(max(map_)), 'rx')
% figure; surf(map_2); hold on; 
% plot3(camera_loc_(1),camera_loc_(2),max(max(map_2)), 'rx')

% vec = [1;1];
% for row_ = 1:size(map_,1)
%     for col_ = 1:size(map_,2)
%         map_3(row_,col_) = abs(cos(atan2(vec(2),vec(1)) - map_2(row_,col_)))  ;    
%     end
% end


% figure; surf(map_3); hold on; 
% daspect([80 80 4])
% plot3(c_col,c_row,max(max(map_3)), 'co', 'LineWidth',8)
% plot3(c_col,c_row,max(max(map_3)), 'ro', 'LineWidth',5)
    % cost function factor normal to ray from camera - test__oriented_cost.m

% f_p_1_ = plot3([70-60 70],[0 60],[1 1].*max(max(map_3)) , 'm', 'LineWidth',4)    
% f_p_2_ = plot3([70-60 70],[0 60],[1.001 1.001].*max(max(map_3)) , 'c', 'LineWidth',2)      
% 
% f_p_1_ = plot3([60-60 60],[0 60],[1 1].*max(max(map_3)) , 'm', 'LineWidth',4)    
% f_p_2_ = plot3([60-60 60],[0 60],[1.001 1.001].*max(max(map_3)) , 'c', 'LineWidth',2)      

% map__ = map_3 ;
map__ = map_2 ;
end