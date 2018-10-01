%{
With tag 970 rotated ~ +45degrees around its Boof-Y axis / FLU-Z axis = +45 yaw
%}

%%
f1=figure_named('corrected coord system, using target-to-sensor'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
draw_axes_direct(eye(3),[0;0;0],'',1) ;
draw_axes_direct_c(eye(3),[0;0;0],'',0.8,'k') ;
axis equal
f2=figure_named('Boofcv coord system, using target-to-sensor'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
draw_axes_direct(eye(3),[0;0;0],'',1) ;
draw_axes_direct_c(eye(3),[0;0;0],'',0.8,'k') ;
axis equal

%%

 qx = 0.9574800540005618, qy = -0.0016850074197882006, qz = 0.2884863964154275, qw = -0.002169337299142648  % tag 890
 
 
 rotation_matrix= [ 0.9301419314365782, 0.02029310694458821, -0.36663902846528307; 0.024853384961616876, -0.9996612877724927, 0.007721333109716206; -0.36635815350471024, -0.016294156608337527, -0.9303312333900449 ]  % tag 1170
 tag = 1170
 offs = -0.3
 
 rotation_matrix= [ 0.9420743564408386, 0.07338272006805321, 0.32727799090680165; 0.009599026055687694, -0.9812713844729299, 0.19239108272828442; 0.3352667082120756, -0.17810515548403727, -0.9251350106631171 ]  % 
 tag = 970
 offs = 0

 
 rotation_matrix= [ 0.8335667398513341, -0.004497054066905567, 0.5524004586514547; -0.0019410909024622197, -0.9999845349584702, -0.005211718526601848; 0.5524153531354162, 0.0032720557164380043, -0.8335625778978216 ]  % 
 tag= 890
 offs = -0.5
  
 
 R = rotation_matrix
 figure(f1)
    %     draw_axes_direct(R,[0;-1;0],'',1.1)
    %     draw_axes_direct_c(R,[0;-1;0],'',0.9,'m')
    %     draw_axes_direct_c(R,[0;-1;0],'',0.9,'g:')

R = rotation_matrix
R = [  R(:,3)  -R(:,1)  -R(:,2)  ]
 R_inv = inv(R)

R_inv_b = [R_inv(:,3),R_inv(:,1),R_inv(:,2)]
     offs = offs+1
    draw_axes_direct(R_inv_b,[0;offs;0],'',1.1)
    draw_axes_direct_c(R_inv_b,[0;offs;0],'',0.9,'c')
    draw_axes_direct_c(R_inv_b,[0;offs;0],'',0.9,'b:'); axis equal
    text(0,offs,0,strcat(tag,' - this one'))
% %              
% %              R2 = rotx(deg2rad(90))*R*rotz(deg2rad(90))
% %              R2_inv = [ R2 R2() R2(:,3) ]
%     %              offs = offs+1
%     %              draw_axes_direct(R2,[0;offs;0],'',1.1)
%     %              draw_axes_direct_c(R2,[0;offs;0],'',0.9,'c')
%     %              draw_axes_direct_c(R2,[0;offs;0],'',0.9,'b:')
% % 
%     % R2 = rotz(deg2rad(90))*rotx(deg2rad(90))*R
%         % offs = offs+1
%         % draw_axes_direct(R2,[0;offs;0],'',1.1)
%         % draw_axes_direct_c(R2,[0;offs;0],'',0.9,'c')
%         % draw_axes_direct_c(R2,[0;offs;0],'',0.9,'b:')

 rotation_matrix= Type = dense real , numRows = 3 , numCols = 3  % tag 1170
10-01 15:43:09.640 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame x = 0.39448975694011446, y = 0.21886274304779818, z = 0.7900070379278531  % tag 1170
10-01 15:43:09.640 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame qx = 0.9823612437132172, qy = 0.011489279578955172, qz = -0.18653962243037622, qw = 0.006111674771308625  % tag 1170
 rotation_matrix= [ 0.9420743564408386, 0.07338272006805321, 0.32727799090680165; 0.009599026055687694, -0.9812713844729299, 0.19239108272828442; 0.3352667082120756, -0.17810515548403727, -0.9251350106631171 ]  % tag 970
 rotation_matrix= Type = dense real , numRows = 3 , numCols = 3  % tag 970
10-01 15:43:10.790 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame x = 0.05378160469026966, y = 0.35777271965312163, z = 0.8792181302931656  % tag 970
10-01 15:43:10.790 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame qx = 0.9808772542444959, qy = 0.021149880314957497, qz = 0.1688653438164368, qw = 0.09442981693404903  % tag 970
 rotation_matrix= [ 0.8335667398513341, -0.004497054066905567, 0.5524004586514547; -0.0019410909024622197, -0.9999845349584702, -0.005211718526601848; 0.5524153531354162, 0.0032720557164380043, -0.8335625778978216 ]  % tag 890
 rotation_matrix= Type = dense real , numRows = 3 , numCols = 3  % tag 890
10-01 15:43:11.475 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame x = -0.6171742939425641, y = 0.19152379563624053, z = 0.6762291624890799  % tag 890
10-01 15:43:11.475 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame qx = 0.9574854897996661, qy = -0.0016810032731448587, qz = 0.2884680299484305, qw = -0.002215118227226322  % tag 890
10-01 15:43:12.370 8979-9484/william.chamberlain.androidvosopencvros I/System.out: onCameraFrame: 3D Location: sensorToTarget_boofcvFrame : BoofCV frame

%%
% 970, row-major
f1=figure_named('target-to-sensor rotation_matrix'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
draw_axes_direct(eye(3),[0;0;0],'',1) ;
draw_axes_direct_c(eye(3),[0;0;0],'',0.8,'k') ;
axis equal
R = [0.803   0.018   0.596  
0.029  -1.000  -0.009  
0.595   0.025  -0.803 ]
draw_axes_direct(R,[0;-1;0],'',1.1)
draw_axes_direct_c(R,[0;-1;0],'',0.9,'m')
draw_axes_direct_c(R,[0;-1;0],'',0.9,'g:')

offs = 0
offs = offs+1
R = [  R(:,3)  -R(:,1)  -R(:,2)  ]
draw_axes_direct(R,[0;1;0],'',1.1)
draw_axes_direct_c(R,[0;1;0],'',0.9,'c')
draw_axes_direct_c(R,[0;1;0],'',0.9,'b:')

offs = offs+1
R2 = rotx(deg2rad(90))*R
draw_axes_direct(R2,[0;offs;0],'',1.1)
draw_axes_direct_c(R2,[0;offs;0],'',0.9,'c')
draw_axes_direct_c(R2,[0;offs;0],'',0.9,'b:')

quat = Quaternion(R2)
f1=figure_named('target-to-sensor rotation_matrix'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
quat.plot
quat.
quat2 = quat.inv  %  
hold on ; quat2.plot; axis equal
draw_axes_direct(eye(3),[0;0;0],'',1)

%%

% qx = 0.9932463793598583, qy = 0.017020551944696514, qz = -0.032104290761996425, qw = 0.11018731875628739
% qx = 0.9587030951360446, qy = -0.001134628319124001, qz = -0.2840646678583406, qw = 0.013941035459256691
% quat = Quaternion(qw,[qx,qy,qz])
% figure;quat.plot
% 

