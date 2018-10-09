%{
With tag 970 rotated ~ +45degrees around its Boof-Y axis / FLU-Z axis = +45 yaw
%}

%%
f1=figure_named('corrected coord system, using target-to-sensor'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
draw_axes_direct(eye(3),[0;0;0],'',1) ;
draw_axes_direct_c(eye(3),[0;0;0],'',0.8,'k') ;
axis equal
% f2=figure_named('Boofcv coord system, using target-to-sensor'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
% draw_axes_direct(eye(3),[0;0;0],'',1) ;
% draw_axes_direct_c(eye(3),[0;0;0],'',0.8,'k') ;
% axis equal
%%
tag=890
rotation_matrix= [ 0.7163985644379797, 0.44831923377998917, 0.5345867202747601; 0.31342418961188356, -0.8913541366103497, 0.3274951610539492; 0.623328464164228, -0.06706465366004288, -0.7790789164084234 ]  % tag 890
x = -0.6941013676029206, y = -0.00876846633656539, z = 0.6914153134782504  % tag 890
qx = 0.9201673241124074, qy = 0.20695785522667026, qz = 0.3145936489202961, qw = 0.1071978444503515  % tag 890

%%
tag=970
rotation_matrix= [ 0.821112354234057, 0.5205186759388758, 0.23416833629473627; 0.32551920193452244, -0.764074518535272, 0.5569805914904074; 0.46884085881684495, -0.38111735479236836, -0.7968298507083876 ]  % tag 970
tag=970
x = -0.09397875129831718, y = 0.3303254164054412, z = 0.8318176918177179  % tag 970
qx = 0.9195130128874899, qy = 0.23002335638966048, qz = 0.19113628226531698, qw = 0.25505292832586585  % tag 970

%%
tag=1170
rotation_matrix= [ 0.8347148679411788, 0.4466786726609592, -0.3220702603901575; 0.3605833270027211, -0.8853788828146215, -0.29340057626661425; -0.4162099872967837, 0.12877265725213705, -0.9001037991357634 ]  % tag 1170
tag=1170
x = 0.47548247739410204, y = -0.16648381110468585, z = 0.7651201542871505  % tag 1170
qx = 0.9513408366473558, qy = 0.21213795533800814, qz = -0.19401044800324488, qw = -0.11094163554634705  % tag 1170
%%
%        THIS
BoofCV_inv_to_VOS = [ 0 0 1 ; - 1 0 0  ; 0 -1 0 ]  ;

R = rotation_matrix'  % - BoofCV is row-major
posn = [x;y;z]
BoofCV = rt2tr(R,posn)
BoofCV_inv = tr_invert(BoofCV)
        % BoofCV_inv_posn
        % VOS_posn
        % VOS_posn_2 = BoofCV_inv_to_VOS *  BoofCV_inv_posn
            % BoofCV_inv_orientation
            % VOS_orientation   % tried by hand 
            % VOS_orientation_2  =  BoofCV_inv_to_VOS * BoofCV_inv_orientation
            % draw_axes_direct(  VOS_orientation_2  ,  VOS_posn_2  ,'',0.2)
            % draw_axes_direct_c(  VOS_orientation_2  ,  VOS_posn_2  ,'',0.15,'r')
            % draw_axes_direct_c(  VOS_orientation_2  ,  VOS_posn_2  ,'',0.15,'m:')
            
BoofCV_inv_orientation = BoofCV_inv(1:3,1:3)  ;     % 1) see if we can get rot, trans of inverted transform straight from BoofCV
BoofCV_inv_posn = BoofCV_inv(1:3,4)  ;                  % 1) see if we can get rot, trans of inverted transform straight from BoofCV
VOS_posn_2 = BoofCV_inv_to_VOS *  BoofCV_inv_posn

VOS_orientation_2  =  BoofCV_inv_to_VOS * BoofCV_inv_orientation

figure(f1)
%   draw_axes_direct_c(  VOS_orientation_2  ,  VOS_posn_2  ,'',0.2,'c')                %   correct vectors as basis vectors, but swapped x->z etc.
%   draw_axes_direct(  VOS_orientation_2  ,  VOS_posn_2  ,'',0.15)                          %   correct vectors as basis vectors, but swapped x->z etc.

VOS_orientation_2_2  =  [ VOS_orientation_2(1:3,3) , VOS_orientation_2(1:3,1) , VOS_orientation_2(1:3,2)  ]
draw_axes_direct(  VOS_orientation_2_2  ,  VOS_posn_2  ,'',0.2)
draw_axes_direct_c(  VOS_orientation_2_2  ,  VOS_posn_2  ,'',0.15,'r')
draw_axes_direct_c(  VOS_orientation_2_2  ,  VOS_posn_2  ,'',0.15,'m:')
text( VOS_posn_2(1) , VOS_posn_2(2) , VOS_posn_2(3)  , int2str(tag) ) 
axis equal
%%

%temp__plot_boofcv_StoT_inVOS_transl(x,y,z,'r')
 figure(f1)
R = rotation_matrix
R = [  R(:,3)  -R(:,1)  -R(:,2)  ]
    %     offs = offs+1
    %     draw_axes_direct(R,[0;offs;0],'',1.1)
    %     draw_axes_direct_c(R,[0;offs;0],'',0.9,'c')
    %     draw_axes_direct_c(R,[0;offs;0],'',0.9,'b:'); axis equal
% %              R2 = rotx(deg2rad(90))*R
%     %              offs = offs+1
%     %              draw_axes_direct(R2,[0;offs;0],'',1.1)
%     %              draw_axes_direct_c(R2,[0;offs;0],'',0.9,'c')
%     %              draw_axes_direct_c(R2,[0;offs;0],'',0.9,'b:')
% %              
% %              R2 = rotx(deg2rad(90))*R*rotz(deg2rad(90))
%     %              offs = offs+1
%     %              draw_axes_direct(R2,[0;offs;0],'',1.1)
%     %              draw_axes_direct_c(R2,[0;offs;0],'',0.9,'c')
%     %              draw_axes_direct_c(R2,[0;offs;0],'',0.9,'b:')            
 R_inv = inv(R)
    %      offs = offs+1
    %     draw_axes_direct(R_inv,[0;offs;0],'',1.1)
    %     draw_axes_direct_c(R_inv,[0;offs;0],'',0.9,'c')
    %     draw_axes_direct_c(R_inv,[0;offs;0],'',0.9,'b:'); axis equal    
R_inv_b = [R_inv(:,3),R_inv(:,1),R_inv(:,2)]
[vos_x,vos_y,vos_z] = temp__boofcv_TtoS_to_VOS_StoT_transl( x , y , z )  ;
vos_y = -vos_y;
vos_posn = [vos_x;vos_y;vos_z]  ;
%%


%%

f1=figure_named('corrected coord system, using target-to-sensor'); hold on; grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z') ;
draw_axes_direct(eye(3),[0;0;0],'',1) ;
draw_axes_direct_c(eye(3),[0;0;0],'',0.8,'k') ;
axis equal


rotation_matrix= [ 0.7785890825030636, -0.03519203340832367, 0.6265465356951749; 0.12035697786739716, -0.9715164833818581, -0.20413211505291018; 0.615884111246316, 0.2343442836980137, -0.7521765206468494 ]  % 
tag = 890
x = -0.7022775705840073, y = -0.039684818996411575, z = 0.7099923642668718  % tag 890
qx = 0.9357192536401839, qy = 0.02275387198878306, qz = 0.3319453570363446, qw = -0.11714956089797769  % tag 890
R = rotation_matrix'  % - BoofCV is row-major
posn = [x;y;z]
BoofCV = rt2tr(R,posn)
BoofCV_inv = tr_invert(BoofCV)

draw_axes_direct(BoofCV_inv(1:3,1:3),BoofCV_inv(1:3,4),'',0.5)
text( BoofCV_inv(1,4) , BoofCV_inv(2,4) , BoofCV_inv(3,4) , int2str(tag) ) 


rotation_matrix= [ 0.9343240049907411, -0.020448102404301623, -0.3558377843991895; -0.10490697432189658, -0.9699062002137573, -0.21971911506634081; -0.34063643439404107, 0.24261870887253148, -0.9083517940029293 ]  % 
tag= 1170
x = 0.4514851779414082, y = -0.03827669108445164, z = 0.850997595072819  % tag 1170
qx = 0.9762917083545557, qy = -0.03209980061632192, qz = -0.17834685392521407, qw = -0.11839131173153604  % tag 1170
R = rotation_matrix'  % - BoofCV is row-major
posn = [x;y;z]
BoofCV = rt2tr(R,posn)
BoofCV_inv = tr_invert(BoofCV)

draw_axes_direct(BoofCV_inv(1:3,1:3),BoofCV_inv(1:3,4),'',0.5)
text( BoofCV_inv(1,4) , BoofCV_inv(2,4) , BoofCV_inv(3,4) , int2str(tag) ) 


rotation_matrix= [ 0.9548605224898966, 0.10554439593533488, 0.2776720423034461; 0.033647592435348546, -0.967164024562726, 0.251915837364258; 0.2951427148239071, -0.23120149237899804, -0.9270580606456631 ]  % 
tag =970
x = 0.0799522090033552, y = 0.4074158340095585, z = 0.8252527519752474  % tag 970
qx = 0.9809539499510522, qy = 0.035473629617789104, qz = 0.1459841099462252, qw = 0.12312436525877762  % tag 970
R = rotation_matrix'  % - BoofCV is row-major
posn = [x;y;z]
BoofCV = rt2tr(R,posn)
BoofCV_inv = tr_invert(BoofCV)

draw_axes_direct(BoofCV_inv(1:3,1:3),BoofCV_inv(1:3,4),'',0.5)
text( BoofCV_inv(1,4) , BoofCV_inv(2,4) , BoofCV_inv(3,4) , int2str(tag) ) 


%%
f2=figure_named('VOS'); grid on; hold on; xlabel('x'); ylabel('y'); zlabel('z');
draw_axes_direct( eye(3), [0;0;0],'',0.2 )  ; axis equal;

figure(f2)

%%

     offs = offs+1
    draw_axes_direct(  R_inv_b  ,  vos_posn  ,'',1.1)
    draw_axes_direct_c(  R_inv_b  ,  vos_posn  ,'',0.9,'c')
    draw_axes_direct_c(  R_inv_b  ,  vos_posn  ,'',0.9,'b:'); axis equal
    text(vos_x,vos_y,vos_z,strcat(int2str(tag),' - this one'))
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

