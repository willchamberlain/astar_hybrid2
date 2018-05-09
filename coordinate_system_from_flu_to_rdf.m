function rdf_coordinate_system__ = coordinate_system_from_flu_to_rdf(flu_coordinate_system_) 
    change_of_basis_rdf_to_flu = ...
        [   0 0 1 ; 
           -1 0 0 ; 
           0 -1 0 ];
    rdf_coordinate_system__ = flu_coordinate_system_*change_of_basis_rdf_to_flu;
end

%{

%--  point the camera:  
%--     direction/x_axis plus z_axis ,  
%--     then invert/flip the y_axis_calculated and z_axis_calculated, 
%--     then convert _the axes/coordinate system_ from FLU to RDF 
    flu_coord_system = coordinate_system_looking_at( [ 5 2  -1  ]' , [ 5 2 10 ]'  )
    flu_coord_system_flipped = [ flu_coord_system(:,1) flu_coord_system(:,2)*-1 flu_coord_system(:,3)*-1 ]   ;
    rdf_coord_system_flipped = coordinate_system_from_flu_to_rdf(flu_coord_system_flipped)


figure
hold on ; plot3( 0 , 0 , 0 , 'bo' ); hold on ; grid on;
xlabel('x');ylabel('y');zlabel('z')

flu_coord_system = coordinate_system_looking_at( [ 5 2  -1  ]' , [ 5 2 10 ]'  )
temp_camera_flu = CentralCamera('Default')
temp_camera_flu.T=  [ flu_coord_system [ 0 0 0 ]' ; [ 0 0 0 1 ] ]  ;
hold on; temp_camera_flu.plot_camera;  hold on; grid on; axis equal; 
draw_axes_direct_c(flu_coord_system, [0 0 0]', '', 0.65,'k')
draw_axes_direct(flu_coord_system, [0 0 0]', '', 0.6)
draw_axes_direct_c(flu_coord_system, [0  -0.01 0]', '', 0.65,'k')
draw_axes_direct(flu_coord_system, [0 -0.01 0]', '', 0.6)

rdf_coord_system = coordinate_system_from_flu_to_rdf(flu_coord_system)
temp_camera_rdf = CentralCamera('Default')
temp_camera_rdf.T=  [ rdf_coord_system [ 0 0 0 ]' ; [ 0 0 0 1 ] ]  ;
hold on; temp_camera_rdf.plot_camera;  hold on; grid on; axis equal; 
draw_axes_direct_c(rdf_coord_system, [0 0 0]', '', 0.65, 'c')
draw_axes_direct(rdf_coord_system, [0 0 0]', '', 0.6)
draw_axes_direct_c(rdf_coord_system, [0 0.01 0]', '', 0.65, 'c')
draw_axes_direct(rdf_coord_system, [0 0.01 0]', '', 0.6)



figure
hold on ; plot3( 0 , 0 , 0 , 'bo' ); grid on;
xlabel('x');ylabel('y');zlabel('z')
flu_coord_system_flipped = [ flu_coord_system(:,1) flu_coord_system(:,2)*-1 flu_coord_system(:,3)*-1 ]   ;
temp_camera_flu_flipped = CentralCamera('Default')
temp_camera_flu_flipped.T=  [ flu_coord_system_flipped [ 0 0 0 ]' ; [ 0 0 0 1 ] ]  ;
hold on; temp_camera_flu_flipped.plot_camera;  hold on; grid on; axis equal; 
draw_axes_direct_c(flu_coord_system_flipped, [0 0 0]', '', 0.65,'k')
draw_axes_direct(flu_coord_system_flipped, [0 0 0]', '', 0.6)
draw_axes_direct_c(flu_coord_system_flipped, [0  -0.01 0]', '', 0.65,'k')
draw_axes_direct(flu_coord_system_flipped, [0 -0.01 0]', '', 0.6)
rdf_coord_system_flipped = coordinate_system_from_flu_to_rdf(flu_coord_system_flipped)
temp_camera_rdf_flipped = CentralCamera('Default')
temp_camera_rdf_flipped.T=  [ rdf_coord_system_flipped [ 0 0 0 ]' ; [ 0 0 0 1 ] ]  ;
hold on; temp_camera_rdf_flipped.plot_camera;  hold on; grid on; axis equal; 
draw_axes_direct_c(rdf_coord_system_flipped, [0 0 0]', '', 0.65, 'c')
draw_axes_direct(rdf_coord_system_flipped, [0 0 0]', '', 0.6)
draw_axes_direct_c(rdf_coord_system_flipped, [0 0.01 0]', '', 0.65, 'c')
draw_axes_direct(rdf_coord_system_flipped, [0 0.01 0]', '', 0.6)

%}


