function check_se3(se3_, matrix_name_) 

% 
% check_se3 Checks that the argument is a valid SE3 transformation
% matrix in Euclidean3-space: throws an exception if not.
% 
%   se3_ : SE3 matrix to check, in homogeneous coordinates: 4x4.
%
    se3_scaled = se3_  ;

    rot_tolerance = 1e-3;
    scale_tolerance = 1e-3;
    
    if    se3_scaled(4,4) > 1.0+scale_tolerance ...
       || se3_scaled(4,4) < 1.0-scale_tolerance 
        display(se3_scaled);
        error('scale is not 1.0 for %s', matrix_name_);
%         scale_tolerance_exception_ = MException('check_se3','scale is not 1.0');
%         throw(scale_tolerance_exception_)
    end

    col_norms = norm_2(se3_,1)  ;
    se3_scaled = se3_./repmat(col_norms,4,1)  ;
    
%     se3_scaled = se3_ ;

    if     abs(dot(se3_scaled(1:3,1),se3_scaled(1:3,2))) > rot_tolerance ...
        || abs(dot(se3_scaled(1:3,1),se3_scaled(1:3,3))) > rot_tolerance             
            display(se3_scaled);
            error('rotation columns are not orthogonal for %s',matrix_name_);
%             rotation_tolerance_exception_ = MException('check_se3','rotation columns are not orthogonal');
%             throw(rotation_tolerance_exception_)        
    end
    
    row_norms = norm_2(se3_scaled,2)  ;
    se3_scaled = se3_scaled./repmat(row_norms,1, 4 )   ;
    if     abs(dot(se3_scaled(1,1:3),se3_scaled(2,1:3))) > rot_tolerance ...
        || abs(dot(se3_scaled(1,1:3),se3_scaled(3,1:3))) > rot_tolerance
            display(se3_scaled);
            error('rotation rows are not orthogonal for %s',matrix_name_);
%             rotation_tolerance_exception_ = MException('check_se3','rotation rows are not orthogonal');
%             throw(rotation_tolerance_exception_)        
    end