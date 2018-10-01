
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

%start for this tag 1050---------------------------------------------------------
% targetToSensor_boofcvFrame : BoofCV frame :
x = 1.151754256375162, y = -5.175813878951355E-4, z = 2.1964812499236466
figure(f1); temp__plot_boofcv_StoT_inVOS_transl(x,y,z,'r')
figure(f2); plot3([0 x],[0 y],[0 z], 'r')
% targetToSensor_boofcvFrame : BoofCV frame : 
qx = 0.9985205445014761, qy = 0.005334758046378224, qz = 0.02208890205763316, qw = 0.04939982764090095
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(qw,qx,qy,qz,  x,y,z)
% sensorToTarget_boofcvFrame : BoofCV frame : x = -1.2462925827228704, y = -0.22748013849452137, z = 2.132152809034899
% sensorToTarget_boofcvFrame : BoofCV frame : qx = 0.9985205445014761, qy = 0.005334758046378224, qz = 0.02208890205763316, qw = -0.04939982764090095
%end for this tag 1050---------------------------------------------------------

%start for this tag 890---------------------------------------------------------
% targetToSensor_boofcvFrame : BoofCV frame :
x = 0.002344306683341463, y = -0.505337154586377, z = 1.9227354918618584
figure(f1); temp__plot_boofcv_StoT_inVOS_transl(x,y,z,'b')
figure(f2); plot3([0 x],[0 y],[0 z], 'b')
% targetToSensor_boofcvFrame : BoofCV frame : 
qx = 0.9927890807358982, qy = 0.007345479449335729, qz = 0.04736110849888732, qw = 0.10987634188025318
figure(f1); temp__plot_boofcv_StoT_inVOS_rot(qw,qx,qy,qz,  x,y,z)

quat_TtoS = Quaternion(qw,[qx,qy,qz])

TtoS = rt2tr(quat_TtoS.R,[x;y;z]) ;
TtoS(1:3,4)'
TtoS(1:3,1:3)
StoT = tr_invert(TtoS) ;
StoT(1:3,4)'
StoT(1:3,1:3)

quat_StoT = quat_TtoS.inv
quat_StoT_FLU = Quaternion(quat_StoT.s , [quat_StoT.v(3),-1*quat_StoT.v(1),-1*quat_StoT.v(2)] )
figure(f1);  quat_StoT_FLU.plot
% sensorToTarget_boofcvFrame : BoofCV frame :
x = -0.16741258559959407, y = -0.913908018676378, z = 1.75756310464985
% sensorToTarget_boofcvFrame : BoofCV frame : qx = 0.9927890807358982, qy = 0.007345479449335729, qz = 0.04736110849888732, qw = -0.10987634188025318
%end for this tag 890---------------------------------------------------------

%start for this tag 1050---------------------------------------------------------
% targetToSensor_boofcvFrame : BoofCV frame :
x = 1.1515403492617298, y = -5.874823151644962E-4, z = 2.1962449467310945
figure(f1); temp__plot_boofcv_StoT_inVOS_transl(x,y,z,'r')
figure(f2); plot3([0 x],[0 y],[0 z], 'r')
% targetToSensor_boofcvFrame : BoofCV frame : qx = 0.9985963982525066, qy = 0.005531493497900394, qz = 0.020958684122258486, qw = 0.04832566126466703
% sensorToTarget_boofcvFrame : BoofCV frame :
x = -1.241207956102232, y = -0.22345490961342238, z = 2.1351833879630435
% sensorToTarget_boofcvFrame : BoofCV frame : qx = 0.9985963982525066, qy = 0.005531493497900394, qz = 0.020958684122258486, qw = -0.04832566126466703
%end for this tag 1050---------------------------------------------------------
%start for this tag 890---------------------------------------------------------
% targetToSensor_boofcvFrame : BoofCV frame :
x = 0.0023631869939094515, y = -0.505585043289966, z = 1.9235960116077677
figure(f1); temp__plot_boofcv_StoT_inVOS_transl(x,y,z,'b')
figure(f2); plot3([0 x],[0 y],[0 z], 'b')
% targetToSensor_boofcvFrame : BoofCV frame : qx = 0.9927912851066389, qy = 0.007269902061350136, qz = 0.0464090649241382, qw = 0.11026700065383918
% sensorToTarget_boofcvFrame : BoofCV frame :
x = -0.16405357370378035, y = -0.915705158961467, z = 1.7579573155600134
% sensorToTarget_boofcvFrame : BoofCV frame : qx = 0.9927912851066389, qy = 0.007269902061350136, qz = 0.0464090649241382, qw = -0.11026700065383918
%end for this tag 890---------------------------------------------------------

%start for this tag 1050---------------------------------------------------------
% targetToSensor_boofcvFrame : BoofCV frame :
x = 1.150288817412657, y = -4.7041979738767593E-4, z = 2.1937347647449728
figure(f1); temp__plot_boofcv_StoT_inVOS_transl(x,y,z,'r')
figure(f2); plot3([0 x],[0 y],[0 z], 'r')
% targetToSensor_boofcvFrame : BoofCV frame : qx = 0.9985953504408868, qy = 0.0049531776305195676, qz = 0.02261908982290707, qw = 0.04765678214888687
% sensorToTarget_boofcvFrame : BoofCV frame :
x = -1.2471151743654196, y = -0.21865804249107182, z = 2.1289737445848314
% sensorToTarget_boofcvFrame : BoofCV frame : qx = 0.9985953504408868, qy = 0.0049531776305195676, qz = 0.02261908982290707, qw = -0.04765678214888687
%end for this tag 1050---------------------------------------------------------
    
    