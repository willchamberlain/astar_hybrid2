function camera__ = camera_extrinsics__place_camera_2(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max, gauss_scale_, target_point_)
    cam_pose_xyz = rand(([3,1])) ;
%     cam_pose_xyz = cam_pose_xyz + [ -0.5 -0.5 -1.0 ]' ;    
%     cam_pose_xyz = cam_pose_xyz.*[x_max y_max z_max/2]' ;
    cam_pose_xyz = cam_pose_xyz + [  -0.5   -0.5   -0.5 ]' ;    % zero mean
    cam_pose_xyz = cam_pose_xyz.*  gauss_scale_   ;  % scale 
    cam_pose_xyz = cam_pose_xyz + [  x_max   y_max   z_max]' ; % offset
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
%         

        rot_SO3 = roty(degtorad(15)) * rotz( degtorad(45) )   ; 
        T_FLU_to_RDF = [ ...
             0    -1     0     0
             0     0    -1     0
             1     0     0     0
             0     0     0     1   ] ;
%         rot_SO3 =  T_FLU_to_RDF(1:3,1:3) * rot_SO3  ;
%         rot_SO3 =  rotx(degtorad(90)) * rot_SO3  ;
    end
%     camera = CentralCamera('default');
%     disp(cam_pose_xyz)
%     camera__ = camera.move(  rot_trans_to_SE3_hom(eye(3), cam_pose_xyz)  );
%     camera__ = camera.move(  [   [ rot_SO3, cam_pose_xyz ] ; [ 0 0 0 1 ]   ]  );  % :-- move in world coordinate system ( FLU ) : camera is aligned to 
%     
    camera__ = CentralCamera('default');
    display(camera__.C)   ;
    camera__ = camera__.move(  rot_trans_to_SE3_hom(eye(3), cam_pose_xyz)  );
    display(camera__.C)   ;
%     camera__ = camera__.move(  [   [ rotx(degtorad(-7))*roty(degtorad(30)) *  rotz(degtorad(35)), [ 0 0 0  ]' ] ; [ 0 0 0 1 ]   ]  );
    camera__ = camera__.move(  [   [ rotx(degtorad(-7))*roty(degtorad(90)) *  rotz(degtorad(0)), [ 0 0 0  ]' ] ; [ 0 0 0 1 ]   ]  );
    display(camera__.C)   ;
    camera__ = camera__.move(  [   [ roty(degtorad(90))*rotz(degtorad(-90)) , [ 0 0 0  ]' ] ; [ 0 0 0 1 ]   ]  );
    display(camera__.C)   ;
    
    
end    