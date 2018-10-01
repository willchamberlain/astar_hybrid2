%  bob
%         position= 
          x= -1.56078690903
          y= -0.495811537593
          z= 4.36141058266
%         orientation= 
          qx= 0.967339842533
          qy= -0.0727755657493
          qz= 0.136288213959
          qw= -0.200954892486
        
          
% kate  -  /dev/video3    - one on the right on the big tripod      
%         position= 
          x= 0.941345072105
          y= 0.171864299286
          z= 4.32542522928
%         orientation= 
          qx= 0.975640292077
          qy= -0.00487552187282
          qz= -0.143973332676
          qw= -0.165450685223

% re-aligned - is now inline in world x from -4 to 0  
% kate  -  /dev/video3    - one on the right on the big tripod      
%         position= 
%
          x= 0.0211845742276  % -->  means that  x is left-to-right, and increases to the right  --> R__ --> RDF
          y= 0.170037799387
          z= 4.48091400636
%         orientation: 
          qx= 0.986710914306
          qy= -0.0253146316238
          qz= -0.0511399208783
          qw= -0.152136285969
%}
          
% q= Quaternion (  [  qw  qx qy qz ] )           
% q.plot
          


q= Quaternion (  [qw  rdf_to_flu([qx qy qz ]')' ] )           
figure;  q.plot
figure;  grid on; hold on; axis equal; xlabel('x'); ylabel('y'); zlabel('z'); draw_axes_direct(q.R, rdf_to_flu([x;y;z]),'',1);  plot2_rows([0;0;0],'bo')

tag_to_cam_orientation = q.R
tag_to_cam_vec = tag_to_cam_orientation * rdf_to_flu([x;y;z])
%   offset = negate_to_cam_to_tag*rdf_to_flu([x;y;z]) 


flu_xyz = rdf_to_flu([x;y;z]) ;
figure;  grid on; hold on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');  
plot2_rows([0;0;0],'bo')  ;
draw_axes_direct_c(q.R, (eye(3).*-1)*tag_to_cam_vec  ,'',4.5,'k')  ;
draw_axes_direct(q.R, (eye(3).*-1)*tag_to_cam_vec  ,'',1)  ;  
draw_axes_direct_c(q.R, (eye(3).*-1)*tag_to_cam_vec - [0;x;0] ,'',4.5,'c')  ;
draw_axes_direct_c(q.R, (eye(3).*-1)*tag_to_cam_vec - [0;x;0] + [0;0;y] ,'',6,'m')  ;
draw_axes_direct_c(q.R, (eye(3).*-1)*tag_to_cam_vec + [0;flu_xyz(2);-flu_xyz(3)] ,'',7,'b:')  ;     % ????

%-------------------------------------------------------------------------------------
%-------------------------------------------------------------------------------------



%%
%    THE REST BELOW IS ANOTHER BUNCH OF FLUFF



%{          
% re-aligned - is now inline in world x from -4 to 0  AND on top of the charger   % -->  y is vertical and increases down  -->  _D_  -->  RDF
      pose: 
        position: 
          x: 0.00852576309841
          y: -0.166661916042
          z: 4.23378426321
        orientation: 
          x: 0.983108225288
          y: -0.0424617873635
          z: -0.0281964780694
          w: 0.175784449281
          %}
RDF         FLU         
R  x          -y
D  y          -z
F  z           x

FLU     RDF
F x         z
L y         -x
U z         -y


          
apriltags_ros  URF / +z-y+x  -->  robot FLU +x+y+z
x -->  z
y -->  y
z -->  x

z -->  x
y -->  y
x -->  z

apriltags_ros_to_FLU = [...
    0   0   1
    0   1  0
    1   0   0
    ]  ;

          
tag_to_camera_transl = apriltags_ros_to_FLU * [x;y;z]
tag_to_camera_rot_q = Quaternion (  [  qw   (apriltags_ros_to_FLU * [qx;qy;qz])' ] )
tag_to_camera_rot_q.R


negate_to_cam_to_tag = [ ...
    1   0   0 
    0   -1  0
    0   0   -1
    ];
camera_to_tag_transl = negate_to_cam_to_tag*tag_to_camera_transl

figure; grid on; hold on; axis equal; xlabel('x'),ylabel('y');zlabel('z') ;
plot3_rows([0;0;0]-camera_to_tag_transl, 'bo')
plot3_rows([0;0;0], 'bs')
% zlim([-0.001, 2])
draw_axes_direct(negate_to_cam_to_tag*tag_to_camera_rot_q.R , [0;0;0]-camera_to_tag_transl , '', 0.3 )

draw_axes_direct(negate_to_cam_to_tag*tag_to_camera_rot_q.R , ([0;0;0]-camera_to_tag_transl)+[0;-2*camera_to_tag_transl(2);0] , '', 4.5 )
draw_axes_direct(tag_to_camera_rot_q.R , ([0;0;0]-camera_to_tag_transl)+[0;-2*camera_to_tag_transl(2);0] , '', 4.5 )

plot3_rows([0;0;0],'ro')

figure
hold on; grid on; hold on; axis equal; xlabel('x'),ylabel('y');zlabel('z') ;
tag_to_camera_rot_q3 = Quaternion (  [  qw   (negate_to_cam_to_tag*apriltags_ros_to_FLU * [qx;qy;qz])' ] )
plot3_rows( [ [0;0;0] tag_to_camera_rot_q3.R*camera_to_tag_transl],'b')
plot3_rows( [ [0;0;0] tag_to_camera_rot_q3.R*[camera_to_tag_transl(1);0;0]],'g')
plot3_rows( [ [0;0;0] tag_to_camera_rot_q3.R*[5;0;0]],'r')
tag_to_camera_rot_q3.plot

figure
tag_to_camera_rot_q2 = Quaternion (  [  qw   qx qy qz ] )
tag_to_camera_rot_q2.plot


          