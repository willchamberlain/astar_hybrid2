function [ points_3D__ , points_3D_hom__] = camera_extrinsics__generate_path_3D_data(num_datapoints_, tracks_starts_, tracks_ends_, tracks_heights_)
%{
num_datapoints_ = 3;
tracks_starts_=[  0 0  ]'   ;  % [ x y ] 
tracks_ends_=[  3 6 ]'  ;   % [ x y ] 
tracks_heights_ = [ 0.265  0.645 ];  % [ z for each x y ]
%}
    for ii_ = 1: size(tracks_heights_,2)
        points_3D_1 = ...
        [ linspace(tracks_starts_(1,1), tracks_ends_(1,1), num_datapoints_) ; % x 
          linspace(tracks_starts_(2,1), tracks_ends_(2,1), num_datapoints_) ; % y
          repmat( tracks_heights_(1,ii_), [1 num_datapoints_] )]   % z
        points_3D__(1:3 , ii_ + [0:num_datapoints_-1]*size(tracks_heights_,2) ) = points_3D_1(1:3,:)  ;
    end
    points_3D_hom__ = geo__euclidean_3D_to_hom(points_3D__);
end