cam_pos_base = [ 0.98674; -2.3426; -9.5367e-07 ]
cam_lens_height = 1.65
cam_pos = cam_pos_base  + [ 0 ; 0 ; cam_lens_height ]
cam_pos'

looking_at_pos = [ 2.9665; 0.32552; 0 ]
relative_pos = looking_at_pos - cam_pos

coordinate_system_as_SO3__ = ...
    coordinate_system_looking_at( relative_pos , relative_pos + [ 0 ; 0 ; 10])

cam_rot = coordinate_system_as_SO3__ 
cam_transl = -1*relative_pos + looking_at_pos 
cam_pose = rt2tr(cam_rot,cam_transl)
tr2rpy(cam_pose)
cam_quat = Quaternion(cam_rot)
%   -0.204058779187665  -0.102685271481555  -0.869656045041977   0.437623254614224
[cam_quat.v(1) cam_quat.v(2) cam_quat.v(3) cam_quat.s]    %  --> camera pose  .launch  file


%%