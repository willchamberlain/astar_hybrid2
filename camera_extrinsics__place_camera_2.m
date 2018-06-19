function camera__ = camera_extrinsics__place_camera_2(min_angle_degs_,angle_range_degs_, x_max, y_max, z_max, gauss_scale_, target_point_)
    if max(size(x_max))>1
        if max(size(x_max)) ==2
            display 'camera_extrinsics__place_camera_2: Need a 3D point, not 2D.'
        elseif max(size(x_max)) > 4
            display 'camera_extrinsics__place_camera_2: position vector is too large: need a 3D point as a 3- or 4-element vector.'
        else
            cam_pose_xyz = x_max  ;
        end
    else        
        cam_pose_xyz = camera_extrinsics__place_camera_position_2(x_max, y_max, z_max, gauss_scale_)  ;
    end
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

        rot_SO3 = roty(degtorad(15)) * rotz( degtorad(45) )   ; 
        T_FLU_to_RDF = [ ...
             0    -1     0     0
             0     0    -1     0
             1     0     0     0
             0     0     0     1   ] ;
    end
        
    cam_direction_vector = target_point_ - cam_pose_xyz ;
    cam_rdf_coord_sys = camera_rdf_coordinate_system(cam_direction_vector, [ cam_direction_vector(1:2) ; cam_direction_vector(3)+2 ])  ;   %  vector_along_x_axis_ , vector_in_z_axis_plane_ )
    cam_T =  rt2tr(  cam_rdf_coord_sys , cam_pose_xyz)  ;
    display('camera_extrinsics__place_camera_2: LOCKED VERTICAL')
    camera__ = CentralCamera('default');
    camera__.T = cam_T  ;

end    