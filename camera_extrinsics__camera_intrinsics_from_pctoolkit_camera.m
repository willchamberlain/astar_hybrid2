%{
    Extract the camera intrinsic matrix from an instance of CentralCamera from Peter Corke's Machine Vision Toolkit.
%}
function [ camera_K__ , Focal_length__ , Principal_point__ ] = camera_extrinsics__camera_intrinsics_from_pctoolkit_camera(CentralCamera_) 
    camera_K__  =  CentralCamera_.C;
    camera_K__  =  camera_K__(:,1:3);
    Focal_length__ =  [camera_K__(1,1) , camera_K__(2,2)]  ;
    Principal_point__ =  [ camera_K__(1,3) , camera_K__(2,3) ]  ;
end