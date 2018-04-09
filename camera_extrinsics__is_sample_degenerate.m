%{
True if 
-  2D points are not unique
-  3D points are not unique  
%}
function is_degenerate__ = camera_extrinsics__is_sample_degenerate(points_3D_model_,points_2D_model_) 
    is_degenerate__ = false;
    if size(uniquetol(points_3D_model_', 0.01, 'ByRows',true )',2) < size(points_3D_model_,2)
        is_degenerate__ = true;
    end
    if size(uniquetol(points_2D_model_', 0.01, 'ByRows',true )',2) < size(points_2D_model_,2)
        is_degenerate__ = true;
    end
end