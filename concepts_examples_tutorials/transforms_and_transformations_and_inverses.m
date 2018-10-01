yaw_45_rot = rotz(degtorad(45))

% Rotation matrix columns _are_ the transformed unit basis vectors (of ?the world/an arbitrary? coordinate system)
%x unit vector/basis vector for world
yaw_45_rot  * [1 ; 0 ; 0]
yaw_45_rot  * [0 ; 1 ; 0]
yaw_45_rot  * [0 ; 0 ; 1]

% Positions are vec3
% Unit basis vectors _are_ the origin translated to the end of the basis vectors: 
%  -->  coordinate frame (in 3D) is defined as 4 positions:   the origin and the three basis vectors
[1 ; 0 ; 0]
[0 ; 1 ; 0]
[0 ; 0 ; 1]
[ [0 ; 0 ; 0] ]


yaw_15_rot = rotz(degtorad(15))

cube_side = 10  ;
offset_length = cube_side * 2  ;
long_basis_vec_length = 2*offset_length;
cube_corners = [  0 0 0; 0 1 0; 1 1 0 ; 1 0 0 ;    0 0 1; 0 1 1; 1 1 1 ; 1  0 1 ;]'  
cube_corners = cube_corners.*cube_side;

figure; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z');
plot3_rows([ 0 2 -2 ; 0 2 -2 ; 0 2 0],'bo')
draw_axes_direct(eye(3),[0 0 0]','',0.6)
draw_axes_direct_c(eye(3),[0 0 0]','world',0.5,'k')
long_basis_vec_x_hom = [0 long_basis_vec_length ; 0 0 ; 0 0 ;  1 1]
plot3_rows(long_basis_vec_x_hom)
axis equal
% plot3_rows(cube_corners,'b')
% plot3_rows(sort(cube_corners,'descend'),'b')
for ii_ = 1: 8    
    text( cube_corners(1,ii_) , cube_corners(2,ii_) , cube_corners(3,ii_), int2str(ii_) , 'Color', [0.3 0.3 0.4])
end
plot_cuboid(  cat(3,cube_corners(:,1:4),cube_corners(:,5:8)), cube_corners, 'Color', [0.9 0.3 0.4])



transform = r2tr(yaw_45_rot)
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y45' , 'Color', [0.7 0.3 0.5])
plot3_rows(transform*long_basis_vec_x_hom,  'Color', [0.7 0.3 0.5])


yaw_45_rot_forward_1m = rt2tr(yaw_45_rot,[offset_length;0;0] )
transform = yaw_45_rot_forward_1m
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after) , 'y45\_f1', 'Color', [0.5 0.3 0.6])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.5 0.3 0.6])


transform = r2tr(yaw_45_rot) * r2tr(yaw_45_rot)
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y45\_y45' , 'Color', [0.3 0.3 0.7])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.3 0.3 0.7])


transform = yaw_45_rot_forward_1m * yaw_45_rot_forward_1m
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y45f1\_y45f1' , 'Color', [0.1 0.3 0.9])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.3 0.9])

transform_inv  =  tr_invert(transform)
cube_corners_after_inversion = transform_inv*cube_corners_after
plot_cuboid_hom(  h2e(cube_corners_after_inversion), 'inv_y45f1\_y45f1' , 'Color', [0.9 0.0 0.0])
plot3_rows(transform_inv*transform*long_basis_vec_x_hom, 'Color', [0.9 0.0 0.0])


yaw_15_rot_forward_2m = rt2tr(yaw_15_rot,[2*offset_length;0;0] )

transform = yaw_15_rot_forward_2m
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y45f1\_y15f2' , 'Color', [0.1 0.9 0.9])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.9 0.9])


transform = yaw_15_rot_forward_2m * yaw_45_rot_forward_1m
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y45f1\_y15f2' , 'Color', [0.1 0.9 0.9])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.9 0.9])



transform = yaw_45_rot_forward_1m * yaw_15_rot_forward_2m
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y15f2\_y45f1' , 'Color', [0.1 0.4 0.4])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.4 0.4])

transform =  yaw_15_rot_forward_2m * (yaw_45_rot_forward_1m * yaw_15_rot_forward_2m)
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y15f2\_y45f1\_y15f2\bb' , 'Color', [0.1 0.4 0.4])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.4 0.4])

transform = yaw_15_rot_forward_2m * yaw_15_rot_forward_2m
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y15f2\_y45f1CC' , 'Color', [0.1 0.4 0.4])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.4 0.4])


transform = yaw_15_rot_forward_2m 
cube_corners_after = transform*e2h(cube_corners)
plot_cuboid_hom(  h2e(cube_corners_after), 'y15f2\_y45f1' , 'Color', [0.1 0.4 0.4])
plot3_rows(transform*long_basis_vec_x_hom, 'Color', [0.1 0.4 0.4])



