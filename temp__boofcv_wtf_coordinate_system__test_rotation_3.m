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

x = 0.3003075967815569, y = -0.5118667471125261, z = 1.9060476572623397  % tag 890
qx = 0.9935183681905898, qy = 0.015165960579765606, qz = -0.028360693720563837, qw = 0.10902713771941913  % tag 890
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(0.10902713771941913,0.9935183681905898,0.015165960579765606,-0.028360693720563837,  0.3003075967815569,-0.5118667471125261,1.9060476572623397)  % 890  % tag 890
x = 0.0784717740342796, y = 0.25000185513327583, z = 1.0314387089606747  % tag 1170
qx = 0.959933546024794, qy = 0.0066630529518325165, qz = 0.27964592906346025, qw = 0.016773350883472277  % tag 1170
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(0.016773350883472277,0.959933546024794,0.0066630529518325165,0.27964592906346025,  0.0784717740342796,0.25000185513327583,1.0314387089606747)  % 1170  % tag 1170
x = -0.10201285108498001, y = 0.23356294842405576, z = 0.8797470413210708  % tag 970
qx = 0.9493833417193828, qy = 0.012328067098248565, qz = 0.31375828413000245, qw = 0.00866189166149618  % tag 970
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(0.00866189166149618,0.9493833417193828,0.012328067098248565,0.31375828413000245,  -0.10201285108498001,0.23356294842405576,0.8797470413210708)  % 970  % tag 970
x = 0.30015867180589545, y = -0.511831409517719, z = 1.9057938536248247  % tag 890
qx = 0.9934402972522803, qy = 0.015088446897571776, qz = -0.02821982276341871, qw = 0.10978322353037866  % tag 890
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(0.10978322353037866,0.9934402972522803,0.015088446897571776,-0.02821982276341871,  0.30015867180589545,-0.511831409517719,1.9057938536248247)  % 890  % tag 890
x = 0.07843431584504708, y = 0.249885073411095, z = 1.0311651718085286  % tag 1170
qx = 0.959950249019524, qy = 0.006559057777798477, qz = 0.279620311381879, qw = 0.016278194958979815  % tag 1170
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(0.016278194958979815,0.959950249019524,0.006559057777798477,0.279620311381879,  0.07843431584504708,0.249885073411095,1.0311651718085286)  % 1170  % tag 1170
x = -0.10203030202603006, y = 0.23352061656862247, z = 0.8798399550427571  % tag 970
qx = 0.9494300869650188, qy = 0.012427858223075842, qz = 0.3136057442670364, qw = 0.008916023121436821  % tag 970
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(0.008916023121436821,0.9494300869650188,0.012427858223075842,0.3136057442670364,  -0.10203030202603006,0.23352061656862247,0.8798399550427571)  % 970  % tag 970

%%
% 970, row-major
f1=figure_named('target-to-sensor Rotation MATRIX'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
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
f1=figure_named('target-to-sensor Rotation MATRIX'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
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

