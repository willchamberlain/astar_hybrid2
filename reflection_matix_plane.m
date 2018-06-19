function reflection_matrix_SO3__ = reflection_matix_plane(plane_vector_1_, plane_vector_2_, reflection_origin_)
% plane_vector_1_ :  vector in plane to reflect through
% plane_vector_2_ :  another vector in the plane to reflect through
% return : reflection_matrix_SO3__ : reflection transformation matrix SO3 
plane_normal_vec = cross(plane_vector_1_,plane_vector_2_) ;      %  vector to reflect _along_ . The cross-product is the normal : in the xy-plane
    plane_unit_normal_vec = plane_normal_vec./norm_2(plane_normal_vec,1) ; %  vector to reflect _along_ 
    reflection_rot_SO3 = eye(3) - 2*(plane_unit_normal_vec*plane_unit_normal_vec') ; % Reflection SO3
    reflection_rot_SE3 = r2t(reflection_rot_SO3) ; % Reflection SE3/Transform
            % plot3_rows(mu','ks')
            % A =  [ [ eye(3) -1*[ mu'] ]  ; [ 0 0 0 1 ] ]
            % R_ = A*R_a*inv(A)
            % plot3_rows(R_*euc2hom(points_3D_f1),'ms')
    translation_to_point_of_reflection =  [ [ eye(3)  [ reflection_origin_] ]  ; [ 0 0 0 1 ] ] ; % Translation SE3/Transform
    reflection_matrix_SO3__ = translation_to_point_of_reflection*reflection_rot_SE3*inv(translation_to_point_of_reflection) ; % Reflection SE3/Transform    
 
end