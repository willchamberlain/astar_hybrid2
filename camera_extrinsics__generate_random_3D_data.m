%{
Generates a number of 3D datapoints in a rectangular volume: 
    data points are distributed uniformly within the volume: 
    returns in column-major [ x ; y ;z ] form.
Parameters
- number_of_data_points_  :  the number of datapoints to generate
- points_volume_dimensions_  :  [ x y z ] vector in m of volume boundaries for the generated points, e.g. [ 10 5 2 ] generates points in a volume from 0-10m in x,
0-5m in y, 0-2m in z.
- points_volume_offsets_  :  [ x_offset y_offset z_offset ] vector in m offsetting the boundary of generated points from the zero in that dimension, e.g.
[ 3 1 5] applied to the previous example changes the ranges to 3-13m, 1-6m, 5-7m.
Returns
- points_3D_random__  :  3D datapoints, 3xnumber_of_data_points_, column-major [ x ; y ;z ] 
- points_3D_random_hom__  :  points_3D_random__  augmented to homogeneous representation, 4xnumber_of_data_points_ : [ x ; y ;z ; 1] 
%}
function [ points_3D_random__ , points_3D_random_hom__]=  camera_extrinsics__generate_random_3D_data( number_of_data_points_, points_volume_dimensions_ , points_volume_offsets_)
    points_3D_random__ = [  ...  
        points_volume_offsets_(1)+(randi(  1+10*points_volume_dimensions_(1)  , [1,number_of_data_points_])-1)/10  ;   
        points_volume_offsets_(2)+(randi(  1+10*points_volume_dimensions_(2)  , [1,number_of_data_points_])-1)./10  ;  
        points_volume_offsets_(3)+(randi(  1+10*points_volume_dimensions_(3)  , [1,number_of_data_points_])-1)/10  ;    ]  ;
    points_3D_random_hom__ = geo__euclidean_3D_to_hom(points_3D_random__);
end