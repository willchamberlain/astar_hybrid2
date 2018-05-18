function cam_pose_xyz = camera_extrinsics__place_camera_position_2(x_max, y_max, z_max)
   cam_pose_xyz = rand(([3,1])) ;
    cam_pose_xyz = cam_pose_xyz + [  -0.5   -0.5   -0.5 ]' ;    % zero mean
    cam_pose_xyz = cam_pose_xyz.*  gauss_scale_   ;  % scale 
    cam_pose_xyz = cam_pose_xyz + [  x_max   y_max   z_max]' ; % offset
end    