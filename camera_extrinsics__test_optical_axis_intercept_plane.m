
results001__load('/mnt/nixbig/ownCloud/project_code/results001__0_003_25__2018_05_15_060427.mat')

%%  --> plot_scene 
figure; hold on; grid on;
draw_axes_direct( camera.T(1:3,1:3), l_o, '', 10)  ;
axis equal ;
plot3( 0  , 0 , 0 , 'bo')
plot3_rows(feature_1_positions(:,points_3D_f1_indices), 'rx'); 
plot3_rows(feature_2_positions(:,points_3D_f2_indices), 'bx'); 


%%
% camera optical axis intercept on the floor plain 
% simplified case of the general line-plane intercept 
camera_to_world = camera.T(1:3,1:3)  ; 
camera_axis_z = camera_to_world*[0  0 1.0 ]'  ;

% Line 
l_o =   camera.T(1:3,4)  ; %  camera position is a point on the camera optical axis
l = camera_axis_z  ; % vector in direction of the line 
%d = % scalar, which extends the line from  l_o  parallel to  l  for some distance (in this until it intercepts the plane).

% Plane
p_o =  [ camera.T(1:2,4) ; 0 ]  ; %  a point on the plane: use camera position with z=0
% p = % set of points in the plane
n = [ 0 0 1 ] ; % vector in direction normal to the plane

d = dot( (p_o - l_o ) , n ) / dot( l , n )  ;

z_intercept = d*l + l_o  ;


% try again, with the z=0.1x + 0.1y plane
x = 0:1:10 ; y = 0:1:10 ;
[ X , Y ] = meshgrid(x, y) ;
Z = 0.1*X + 0.1*Y
n = cross(   ([ -10 -10  (0.1*-10+ 0.1*-10) ] - [ 10 10  (0.1*10+ 0.1*10) ])  , ...
    ([ 100 -10  (0.1*100+ 0.1*-10) ] - [ -50 10  (0.1*-50+ 0.1*10) ])     )
figure ; surf(X,Y,Z); grid on; axis equal ; hold on;
 alpha(handle_patch, 0.3)  ;
colormap jet
shading interp
p_o =  [ camera.T(1:2,4) ; sum(camera.T(1:2,4).*[0.1;0.1] )  ]  ; 
plot3_rows(p_o,'ko')
handle_patch = patch( [ -10 -10 10 15 ]' , [ -10 10 10 -9 ]' , ones(4,1)*p_o(3) , 'y' )  ;
alpha(handle_patch,0.4)
d = dot( (p_o - l_o ) , n ) / dot( l , n )  ;
z_intercept = d*l + l_o  ;
plot3_rows(z_intercept,'rs')



%% plot the (z=0)-intercept point 

plot3_rows( d*l + l_o , 'rs' )  ;

% plot a patch of the z=0 plane 
 handle_patch = patch( [ -10 -10 10 15 ]' , [ -10 10 10 -9 ]' , [ 0 0 0 0 ] )  ;
 alpha(handle_patch, 0.3)  ;
figure(gcf)


%%   PCA to derive the shape/moments of interia of the 3D data
[coeff, score, latent, tsquared, explained, mu]  =  pca( [feature_1_positions feature_2_positions]' ) ;
plot3_rows( mu' , 'mo' )
plot3_rows( [ mu'-(coeff(:,1)*10) mu'+(coeff(:,1)*10) ] , 'm')
plot3_rows( [ mu'-(coeff(:,2)*10) mu'+(coeff(:,2)*10) ] , 'c')
plot3_rows( [ mu'-(coeff(:,3)*10) mu'+(coeff(:,3)*10) ] , 'k')

%% PCA gives one set of axes to reflect across/through: try that 
pca_major = coeff(:,1)
pca_major_vert = [ pca_major(1:2) ;pca_major(3)+10 ]
normal_vec = cross(pca_major,pca_major_vert)
pca_major_vert = cross(normal_vec, pca_major)
figure; draw_axes_direct( [pca_major, pca_major_vert, normal_vec]  , [0 0 0]', '', 1  )
hold on; grid on; axis equal




