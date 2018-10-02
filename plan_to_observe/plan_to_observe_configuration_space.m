addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'

%%

floorplan_w = 5 % 5m
floorplan_x = floorplan_w
floorplan_l = 5 % 5m
floorplan_y = floorplan_l
floorplan_scale = 0.1  % 10 cells/metre

orientation_space = 2*pi 
orientation_space_divisions = 8
orientation_scale = orientation_space/orientation_space_divisions   %  45 degree increments 

floorplan_space = zeros(  ceil(floorplan_l*floorplan_scale)  ,  ceil(floorplan_w*floorplan_scale)  ) ;

configuration_space = zeros( size(floorplan_space,1) , size(floorplan_space,2) , orientation_space_divisions)  ;

