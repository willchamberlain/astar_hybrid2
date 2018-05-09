%{
    Extract the camera intrinsic matrix from a default instance of CentralCamera from Peter Corke's Machine Vision Toolkit.
%}
function [ camera_K__ , Focal_length__ , Principal_point__ ] = camera_extrinsics__camera_intrinsics_from_default_pctoolkit_cam()     
    camera_default_ = CentralCamera('default');
    move_trans_x = [ 0 0 0]';
    camera_default_ = camera_default_.move(  [   [ eye(3), move_trans_x ] ; [ 0 0 0 1 ]   ]  );  % :-- move in world coordinate system ( FLU ) : camera is aligned to 
    [ camera_K__ , Focal_length__ , Principal_point__ ] = camera_extrinsics__camera_intrinsics_from_pctoolkit_camera( camera_default_ );    
end