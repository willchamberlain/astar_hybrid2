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
rotation_matrix= [ 0.8238007389108882, -0.009012162834996396, 0.5668078364762699; 0.12205077927697469, -0.9736041599742418, -0.19286924834906732; 0.5535846365712813, 0.22806516744537408, -0.8009558848965128 ]  % 
tag =890
x = -0.684525035213223, y = 0.11227687125892774, z = 0.7652940760855362  % tag 890
qx = 0.9484672877571533, qy = 0.029795075144152163, qz = 0.29531658274081085, qw = -0.11095122130933681  % tag 890
%%
rotation_matrix= [ 0.9212408992286756, 0.002149602724150469, -0.38898661261857115; -0.08567693617251718, -0.9743065162706485, -0.20829372280662448; -0.3794399401702889, 0.22521587768334814, -0.8973868397981367 ]  % 
tag =1170
x = 0.4740318688926797, y = 0.12539536050034295, z = 0.881092719092658  % tag 1170
qx = 0.9737728502193749, qy = -0.021444255051254867, qz = -0.19728074997566072, qw = -0.11129638713800402  % tag 1170
%%
rotation_matrix= [ 0.9703272965205973, 0.09089857282952918, 0.22405889200070772; 0.04304497780975128, -0.9767838870973198, 0.20985797052389596; 0.23793290548469342, -0.19398630715750312, -0.9517127955024072 ]  % 
tag =970
x = 0.10239308362438765, y = 0.503553970293782, z = 0.8044543771572938  % tag 970
qx = 0.9872719963516037, qy = 0.03391759088028925, qz = 0.11698695982278955, qw = 0.10226266904504991  % tag 970
%%
rotation_matrix= [ 0.8240270402303774, -0.009110392372296092, 0.5664772173000314; 0.12252686476240056, -0.9733420356843252, -0.19388772261644177; 0.5531424810848111, 0.22917740359973682, -0.8009438889826321 ]  % 
tag= 890
x = -0.6846062400730695, y = 0.11118191966384633, z = 0.7658600562232809  % tag 890
qx = 0.9484609856100217, qy = 0.029894870245284363, qz = 0.29511485326534986, qw = -0.1115135816430224  % tag 890
%%
rotation_matrix= [ 0.9211897519570154, 0.0022520641111666156, -0.3891071434664395; -0.08576412267030831, -0.9742161016484963, -0.20868038372448683; -0.37954440603434725, 0.2256056637056037, -0.8972447427274169 ]  % 
tag =1170
x = 0.47426901390797854, y = 0.12507461037728373, z = 0.8813571180922468  % tag 1170
qx = 0.9737364371754978, qy = -0.021441135242248874, qz = -0.19734589365126418, qw = -0.11149989639132159  % tag 1170
%%
rotation_matrix= [ 0.9704695276535208, 0.09092084201698983, 0.22343297962224398; 0.04316275309358014, -0.9767501966804568, 0.20999054747790818; 0.2373307241832274, -0.19414544489019192, -0.9518306958630061 ]  % 
tag =970
x = 0.10292369066729495, y = 0.503747899934707, z = 0.8042697698496185  % tag 970
qx = 0.9873006659823774, qy = 0.033952067422428754, qz = 0.11667259014431164, qw = 0.10233356623080452  % tag 970
%%
rotation_matrix= [ 0.8241325102813883, -0.008825237471857247, 0.5663282799585971; 0.12192222800977028, -0.9736742102894169, -0.1925967355237725; 0.5531189526805527, 0.2277732367967003, -0.8013605785067811 ]  % 
tag =890
x = -0.6843539272212676, y = 0.1126450481376712, z = 0.7656425665270764  % tag 890
qx = 0.9485735737249887, qy = 0.029807121363762082, qz = 0.2950343715151032, qw = -0.1107900283026298  % tag 890
%%
rotation_matrix= [ 0.9207706869831522, 0.002939911161134068, -0.39009319260266284; -0.08606142842483114, -0.9738032476438137, -0.210477232535586; -0.3804928022053128, 0.22737324337029216, -0.8963964723654498 ]  % 
tag =1170
x = 0.47504322718852043, y = 0.12328681176496842, z = 0.8807783940762566  % tag 1170
qx = 0.9735207248683019, qy = -0.021345595204185757, qz = -0.19788638678242332, qw = -0.11243994727618939  % tag 1170
%%
rotation_matrix= [ 0.9701538422088245, 0.09125293948656181, 0.22466513633075333; 0.04338030814352385, -0.9768571837948506, 0.20944734740207677; 0.2385784384910485, -0.19345010597943646, -0.9516603307817961 ]  % 
tag =970
x = 0.10172588457630741, y = 0.5034118185028356, z = 0.8048986669075971  % tag 970
qx = 0.9872526724179418, qy = 0.03409290534011598, qz = 0.11730623470667424, qw = 0.10202490827265934  % tag 970
%%

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

