function camera__ = camera_extrinsics__place_camera(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max)
    cam_pose_xyz = rand(([3,1])) ;
    cam_pose_xyz = cam_pose_xyz + [ -0.5 -0.5 -1.0 ]' ;    
    cam_pose_xyz = cam_pose_xyz.*[x_max y_max z_max]' ;
    %cam_pose_xyz(3) = cam_pose_xyz(3)-5;
    if nargin < 1 
        angle_range_degs = 10;
    else
        min_angle_degs = min_angle_degs_;
        angle_range_degs = angle_range_degs_;
    end
    if 0 == angle_range_degs 
       rot_SO3 = eye(3); 
    else
        angle_range = degtorad(angle_range_degs-min_angle_degs);
        rpy =  (rand(([1,3])) - 0.5 ).* angle_range  + min_angle_degs;
        rot_SO3 = rpy2r(degtorad(rpy));        
    end
    camera = CentralCamera('default');
    camera = camera.move(  [   [ rot_SO3, cam_pose_xyz ] ; [ 0 0 0 1 ]   ]  );  % :-- move in world coordinate system ( FLU ) : camera is aligned to 
    camera__ = camera.move(  rot_trans_to_SE3_hom(eye(3), cam_pose_xyz)  );
end    