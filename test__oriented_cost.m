map_ = zeros(60,70);
map_2 = zeros(60,70);
map_3 = zeros(60,70);

camera_loc_ = [40;20] ;
map_(camera_loc_(1),camera_loc_(2)) = 1;

idisp(map_)

figure; surf(map_); hold on; 
plot3(camera_loc_(1),camera_loc_(2),max(max(map_)), 'rx')

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
figure; surf(map_); hold on; 
plot3(camera_loc_(1),camera_loc_(2),max(max(map_)), 'rx')
figure; surf(map_2); hold on; 
plot3(camera_loc_(1),camera_loc_(2),max(max(map_2)), 'rx')

vec = [1;1];
for row_ = 1:size(map_,1)
    for col_ = 1:size(map_,2)
        map_3(row_,col_) = abs(cos(atan2(vec(2),vec(1)) - map_2(row_,col_)))  ;    
    end
end
figure; surf(map_3); hold on; 
daspect([80 80 4])
plot3(c_col,c_row,max(max(map_3)), 'co', 'LineWidth',8)
plot3(c_col,c_row,max(max(map_3)), 'ro', 'LineWidth',5)
    % cost function factor normal to ray from camera - test__oriented_cost.m

f_p_1_ = plot3([70-60 70],[0 60],[1 1].*max(max(map_3)) , 'm', 'LineWidth',4)    
f_p_2_ = plot3([70-60 70],[0 60],[1.001 1.001].*max(max(map_3)) , 'c', 'LineWidth',2)      

f_p_1_ = plot3([60-60 60],[0 60],[1 1].*max(max(map_3)) , 'm', 'LineWidth',4)    
f_p_2_ = plot3([60-60 60],[0 60],[1.001 1.001].*max(max(map_3)) , 'c', 'LineWidth',2)      

%-----

camera_location = camera_loc_ ;
map_3_from_func = oriented_cost(map_, camera_location);
figure; surf(map_3_from_func); hold on; 
daspect([80 80 4])
plot3(c_col,c_row,max(max(map_3_from_func)), 'co', 'LineWidth',8)
plot3(c_col,c_row,max(max(map_3_from_func)), 'ro', 'LineWidth',5)

figure; surf(map_3-map_3_from_func)
max(max(map_3-map_3_from_func))

%-----

map_3_from_func_as_cost_factor = map_3_from_func*0.6;
[ min(map_3_from_func_as_cost_factor(:))  max(map_3_from_func_as_cost_factor(:)) ] ;
map_3_from_func_as_cost_factor = map_3_from_func_as_cost_factor + (1-max(map_3_from_func_as_cost_factor(:))) ; 
[ min(map_3_from_func_as_cost_factor(:))  max(map_3_from_func_as_cost_factor(:)) ] ;

map_3_from_func_adjusted_up = oriented_cost_adjust_up(map_3_from_func)  ;

figure;  surf(map_3_from_func_adjusted_up - map_3_from_func_as_cost_factor);
max(max(map_3_from_func_adjusted_up - map_3_from_func_as_cost_factor))

%-----

