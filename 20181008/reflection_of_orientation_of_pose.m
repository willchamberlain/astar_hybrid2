

% from rviz in ROS format: x,y,z,  w
v=[0.31261, -0.71562, 0.26987]  , s = 0.56332 
v=[0.37375; -0.65649; 0.36686]'  , s = 0.5429
robot_quat = Quaternion([s,v])



figure; hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z'); axis equal

% robot_quat.plot();  
view(3)
[robot_quat.v(1) robot_quat.v(2) robot_quat.v(3) robot_quat.s]
draw_axes_direct(robot_quat.R, [0.02;0.02;0.02], '', 1)
draw_axes_direct_c(robot_quat.R, [0.02;0.02;0.02], '', 0.8,'m')

robot_quat_R = robot_quat.R

% reflect using Quaternion - may add a small twist 
robot_quat_R_rect = [  robot_quat_R(:,3) -robot_quat_R(:,2) robot_quat_R(:,1) ]
robot_quat_rect = Quaternion(robot_quat_R_rect )
% robot_quat_rect.plot()

draw_axes_direct(robot_quat_rect.R, [0;0;0], '', 1)
draw_axes_direct_c(robot_quat_rect.R, [0;0;0], '', 0.8,'c')

%  mirror in the x-z _world_ plane / along the _world_ y-axis
% robot_quat_R_rect = [    1  0  0 ;
%                                         0 -1  0 ;
%                                         0   0  1]   * robot_quat_R_rect 

%robot_quat_rect = Quaternion( [ robot_quat.v(3) -robot_quat.v(2) robot_quat.v(1) robot_quat.s  ] )      %  [s v1 v2 v3]
% robot_quat_rect = Quaternion(robot_quat_R_rect )
%robot_quat_rect.plot()

%  mirror in the x-z _world_ plane / along the _world_ y-axis
% robot_quat_rect_2 =  Quaternion( [ robot_quat_rect.s -robot_quat_rect.v(1) robot_quat_rect.v(2) -robot_quat_rect.v(3) ] )
% robot_quat_rect_2.plot()

%xzplane_normal_quat = Quaternion( [ 0 ,   0 , -1 0 ]  )

% reflect _back_ using reflection ('rotation') matrix
xzplane_unit_x = robot_quat_R_rect * [1;0;0]
xzplane_unit_z = robot_quat_R_rect * [0;0;1]
xzplane_unit_y = robot_quat_R_rect * [0;1;0]
% xzplane_normal_quat = Quaternion( [ 0 ,   xzplane_unit_y(1) , xzplane_unit_y(2)  xzplane_unit_y(3) ]  )
% robot_quat_rect_3 = xzplane_normal_quat  *  robot_quat_rect  *  xzplane_normal_quat 
% % robot_quat_rect_3.plot() 
% robot_quat_rect_3
Px = xzplane_unit_y(1) ; 
Py = xzplane_unit_y(2) ; 
Pz = xzplane_unit_y(3) ; 
R_reflect = ...
             [   -Px^2+Py^2+Pz^2   ,  -2 * Px * Py  ,                -2 * Px * Pz            ;
                 -2 * Py * Px               ,   Px^2-Py^2+Pz^2  ,       -2 * Py * Pz            ;
                 -2 * Pz * Px               ,  -2 * Pz * Py  ,                  Px^2+Py^2-Pz^2  ]
reflection_ = 1/(Px^2+Py^2+Pz^2) * R_reflect * robot_quat_rect.R

draw_axes_direct(reflection_, [-0.02;-0.02;-0.02], '', 0.7)
draw_axes_direct_c(reflection_, [-0.02;-0.02;-0.02], '', 0.5,'y')