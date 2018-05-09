function camera_rdf_coordinate_system__ = camera_rdf_coordinate_system( vector_along_x_axis_ , vector_in_z_axis_plane_ )
% see notes in   /mnt/nixbig/ownCloud/project_code/coordinate_system_from_flu_to_rdf.m
    flu_coord_system = coordinate_system_looking_at( vector_along_x_axis_ , vector_in_z_axis_plane_  )   ;
    flu_coord_system_flipped = [ flu_coord_system(:,1) flu_coord_system(:,2)*-1 flu_coord_system(:,3)*-1 ]   ;
    rdf_coord_system_flipped = coordinate_system_from_flu_to_rdf(flu_coord_system_flipped)   ;
    camera_rdf_coordinate_system__ = rdf_coord_system_flipped   ;
%--  point the camera:  
%--     direction/x_axis plus z_axis ,  
%--     then invert/flip the y_axis_calculated and z_axis_calculated, 
%--     then convert _the axes/coordinate system_ from FLU to RDF 


%{
cam_rdf_ = camera_rdf_coordinate_system( [ 5 2  -1  ]' , [ 5 2 10 ]' )
temp_cam__ = CentralCamera('Default');
temp_cam__.T = r2t(cam_rdf_)
figure; temp_cam__.plot_camera
hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
draw_axes_direct( camera_rdf_coordinate_system( [ 5 2  -1  ]' , [ 5 2 10 ]' ), [ 0 0 0 ]' , '', 10 )
axis equal
plot3( 0 , 0 , 0 , 'bo' )
plot3( 5 , 2 , -1  ,  'rx'  )
plot3( 5*0.1 , 2*0.1 , -1*0.1  ,  'rx'  )
%}

%{
coordinate_system_looking_at( [ 5 2  -1  ]' , [ 5 2 10 ]' )
    flu_coord_system = coordinate_system_looking_at( [ 5 2  -1  ]' , [ 5 2 10 ]' )   ;
    flu_coord_system_flipped = [ flu_coord_system(:,1) flu_coord_system(:,2)*-1 flu_coord_system(:,3)*-1 ]   ;
    rdf_coord_system_flipped = coordinate_system_from_flu_to_rdf(flu_coord_system_flipped)   ;
    camera_rdf_coordinate_system__ = rdf_coord_system_flipped   ;
%}
end