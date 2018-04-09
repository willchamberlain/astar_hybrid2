% uses Peter Corke's Machine Vision Toolkit and Robot Toolkit.

x=0.0
y=0.0
z=0.506100125241867
w=-0.862474731937212
transform_ = quat2tform([w x y z])  % Q, is an N-by-4 matrix containing N quaternions
transform_display_ = transform_;
transform_display_(1:3,4) = [0 0 1]';
[R,t]=tr2rt(transform_display_)
draw_axes_direct(R,t, '', 1); grid on; axis equal; plot3( 0, 0, 0, 'rx')

rpy_ = tr2rpy(transform_) % xuz
rpy_zyx_ = tr2rpy(transform_,'zyx')
rpy_2_zyx_ = rpy_zyx_ + [ 0 20*pi/180 0]  % note : deg2rad(20) 
rot_2_zyx_ = rpy2r(rpy_2_zyx_,'zyx')
quat_2_zyx_ = rotm2quat(rot_2_zyx_)     % quaternion is of the form q = [w x y z]

transform_display_zyx_ = rot_2_zyx_;
transform_display_zyx_(1:3,4) = [0 0 1]';
draw_axes_direct(rot_2_zyx_,[-1 0 1]', '', 1); grid on; axis equal; plot3( 0, 0, 0, 'rx')

 fprintf('\n x: %4.12f , y: %4.12f , z: %4.12f , w: %4.12f \n',quat_2_zyx_(2),quat_2_zyx_(3),quat_2_zyx_(4),quat_2_zyx_(1))

return

rpy_2_ = rpy_ + [ 0 20*pi/180 0]  % note : deg2rad(20) 
rot_2_ = rpy2r(rpy_2_)
quat_2_ = rotm2quat(rot_2_)

transform_display_ = rot_2_;
transform_display_(1:3,4) = [0 0 1]';
draw_axes_direct(rot_2_,[1 0 1]', '', 1); grid on; axis equal; plot3( 0, 0, 0, 'rx')


