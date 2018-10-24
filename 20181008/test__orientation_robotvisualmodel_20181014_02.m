
%% --------------------------------------------------------------------
%  basic check

yawPos90 = rotz(degtorad(90))
offset = [ 0.1 ; 0.15 ; 1.0 ]
tag_posn_on_robot = rt2tr(yawPos90,offset)

figure_named('check offset', 'default'); 
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
plot3(-2,-2,2,'ro')
view(3)

draw_axes_direct(eye(3),offset,'',0.35)
draw_axes_direct_c(eye(3),offset,'',0.25,'k')

draw_axes_direct(eye(3),offset,'',0.35)
draw_axes_direct_c(eye(3),offset,'',0.25,'k')

draw_axes_direct(yawPos90,offset,'',0.35)
draw_axes_direct_c(yawPos90,offset,'',0.25,'m')

%% --------------------------------------------------------------------
%  check that numpy is consistent with Matlab


rotz(degtorad(-45))


rotz(degtorad(-90))

rot_of_170 = roty(degtorad(-90))

inv(rot_of_170)

transl_of_170 = [ 0.0000;  0.0000;  0.640 ]

tr_of_170 = rt2tr(rot_of_170,transl_of_170)


tr_of_170_inverse = tr_invert(tr_of_170)


camera_to_tag_SE3 = ...
[-0.46333843 -0.64230804  0.61053902  0.55896769
 -0.04017632 -0.67302116 -0.73853123  0.57131003
  0.88527022 -0.36671911  0.28603099 -0.52905844
  0.          0.          0.          1.        ]

camera_to_tag_to_base_link = camera_to_tag_SE3*tr_of_170_inverse

figure_named('','default');  view(3)
draw_axes_direct(camera_to_tag_to_base_link(1:3,1:3),camera_to_tag_to_base_link(1:3,4),'',1.2)
draw_axes_direct_c(camera_to_tag_to_base_link(1:3,1:3),camera_to_tag_to_base_link(1:3,4),'',1,'k')
axis equal

draw_axes_direct(camera_to_tag_SE3(1:3,1:3),camera_to_tag_to_base_link(1:3,4),'',1.2)


%%

setenv('ROS_MASTER_URI','http://192.168.43.252:11311')
setenv('ROS_IP','192.168.43.252')
setenv('ROS_HOSTNAME','192.168.43.252')


rosinit()  % global node, and try to connect to localhost:11311

%  start ROS Masters / roscores  -  see  https://au.mathworks.com/matlabcentral/answers/353299-does-the-robotics-system-toolbox-support-connecting-two-ros-master-core-with-different-ports
% m1 = robotics.ros.Core; % Default port of 11311
% m2 = robotics.ros.Core(12000);

% start a node on the VOS Server 's  ROS Master
node1 = robotics.ros.Node('/test_node_1','192.168.43.252',11311)
% attach a subscriber to the node
% feature_detections_2_sub = robotics.ros.Subscriber(node1, '/feature_detections_2') 
%  feature_detections_2_sub.addlistener
% see below  feature_detections_2_sub_w_callback = robotics.ros.Subscriber(node1, '/feature_detections',@{ros_callback,}) 
% feature_detections_2_sub_w_callback

% % start another node - can be on a different master / URI 
% node2 = robotics.ros.Node('/test_node_2','localhost',12000);


% fake two more masters and check communications
master_pioneer1 = robotics.ros.Core(12100); 
master_pioneer2 = robotics.ros.Core(12200); 

clear('pub_pioneer2')
clear('node_pioneer2')
%  node_pioneer2 = robotics.ros.Node('/node_pioneer2__base_pose_ground_truth','192.168.86.29',11311)
node_pioneer2 = robotics.ros.Node('/node_pioneer2__base_pose_ground_truth','192.168.43.252',12200)
pub_pioneer2 = robotics.ros.Publisher(node_pioneer2,'/base_pose_ground_truth','nav_msgs/Odometry')
[node_pioneer2.MasterURI node_pioneer2.NodeURI] 


clear('pub_pioneer1')
clear('node_pioneer1')
%  node_pioneer1 = robotics.ros.Node('/node_pioneer1__base_pose_ground_truth','192.168.86.30',11311)
node_pioneer1 = robotics.ros.Node('/node_pioneer1__base_pose_ground_truth','192.168.43.252',12100)
pub_pioneer1 = robotics.ros.Publisher(node_pioneer1,'/base_pose_ground_truth','nav_msgs/Odometry')
[node_pioneer1.MasterURI node_pioneer1.NodeURI] 

message_ = rosmessage(pub_pioneer1)
message_.Header.FrameId='map'
message_.ChildFrameId=''
message_.Pose.Pose.Position
message_.Pose.Pose.Orientation.

%---
feature_detections_2_sub_w_callback = ...
    robotics.ros.Subscriber(node1, '/feature_detections',{@ros_callback,pub_pioneer1,pub_pioneer2}) ;
clear('feature_detections_2_sub_w_callback')

%%
%--- camera pose: mobile lab, via rviz  ---%
camera_posn = [1.0;-2.3;1.22]
looking_at_posn = [2.98;0.3;0.0]
looking_direction = looking_at_posn - camera_posn
looking_direction_up = looking_direction+[0;0;10]
coordinate_system_as_SO3__ = coordinate_system_looking_at(looking_direction,looking_direction_up)
figure_named('check looking at', 'default')
draw_axes_direct(coordinate_system_as_SO3__, camera_posn, '', 4)
plot3(0,0,0,'bo')
draw_axes_direct(eye(3), [0;0;0], '', 4)
patch([-2,-2,4,4],[-2,4,4,-2],[0,0,0,0],'k','FaceAlpha',0.2)
patch([-2,-2,4,4],[-2,4,4,-2],[1.22 1.22 1.22 1.22],'k','FaceAlpha',0.05)
view(3)

%--- robot pose - from rostopic on one of the local masters via
        % export ROS_MASTER_URI=http://localhost:12100
        % rostopic echo /base_pose_ground_truth
        %{
              x: 1.06550891565
              y: -0.464544294393
              z: -0.00107756625147
            orientation: 
              x: -0.0612446975196
              y: 0.108237510416
              z: 1.08930171575
              w: 0.0
        
              x: 1.04703892431
      y: -0.462027855191
      z: 0.00357342749709
    orientation: 
      x: 0.00149190984326
      y: 0.0236175934392
      z: -0.671254973314
      w: 0.740848664901

        %}
        
 robot_posn= [ 1.06550891565
      -0.464544294393
      -0.00107756625147 ] 
  robot_quat = Quaternion( [  0.740848664901  0.00149190984326 0.0236175934392 -0.671254973314 ])
  draw_axes_direct(robot_quat.R,robot_posn,'',1)
  
     um = [-1.3956   -0.1179   -0.1983    1.9721
    0.0858   -1.3776    0.2657   -1.7856
   -0.1210    0.2988    0.9632    0.8500
         0         0         0    1.0000]

  draw_axes_direct(um(1:3,1:3),um(1:3,4),'',1)
  
  
     x= 1.98398080149
      y= 0.77873450197
      z= -0.10542449187
    robot_posn=[x;y;z]      
      x= -0.00822159588588
      y= 0.0177235003334
      z= -0.477822976822
      w= 0.878238854593

      robot_quat = Quaternion( [  w   x y z ] )
  draw_axes_direct(robot_quat.R,robot_posn,'',1)
  
  norm_2(robot_posn - camera_posn,1)
  camera_posn - robot_posn
  
  %----
  tr_world_to_base_link = [
  -0.483440265246559  -0.874876273300319  -0.029614461914465  -1.949257079932728
   0.370706490873162  -0.173963814383927  -0.912312056759913   1.300299522065660
   0.793008327549493  -0.452026655963053   0.408423425792419  -1.269342101419490
                   0                   0                   0   1.000000000000000 ]
tr_world_to_base_link = [
  -0.997648737162411  -0.064553361982000  -0.023018703157819  -2.225385332786336
   0.064400376642231  -0.997897239532415   0.007327402126570   0.672099350105585
  -0.023443308780666   0.005827760325074   0.999708181662533  -1.434767461423957
                   0                   0                   0   1.000000000000000 ]
               
  draw_axes_direct(tr_world_to_base_link(1:3,1:3),tr_world_to_base_link(1:3,4),'',1)
  draw_axes_direct(tr_world_to_base_link(1:3,1:3),tr_world_to_base_link(1:3,4)+cam_pose(1:3,4)-[cam_pose(1,4)*2;0;0],'',1)
  
  tr_world_to_base_link(1:3,4)+cam_pose(1:3,4)-[cam_pose(1,4)*2;0;0]
  
  plot3(0,0,0,'bo')
  
  %%
  
%--- camera pose: home 20181015_1320  ---%
camera_posn = [-1.76;-0.88;1.44]
looking_at_posn = [0.00;0.32;0.0]
looking_direction = looking_at_posn - camera_posn
looking_direction_up = looking_direction+[0;0;10]
camera_orientation = coordinate_system_looking_at(looking_direction,looking_direction_up)
cam_pose = rt2tr(camera_orientation,camera_posn)
figure_named('check looking at', 'default')
draw_axes_direct(camera_orientation, camera_posn, '', 4)
plot3(0,0,0,'bo')
draw_axes_direct(eye(3), [0;0;0], '', 4)
patch([-2,-2,4,4],[-2,4,4,-2],[0,0,0,0],'k','FaceAlpha',0.2)
patch([-2,-2,4,4],[-2,4,4,-2],[camera_posn(3),camera_posn(3),camera_posn(3),camera_posn(3)],'k','FaceAlpha',0.05)
view(3)


%--- robot pose - from rostopic on one of the local masters via
        % export ROS_MASTER_URI=http://localhost:12100
        % rostopic echo /base_pose_ground_truth
        %{
              x: 1.06550891565
              y: -0.464544294393
              z: -0.00107756625147
            orientation: 
              x: -0.0612446975196
              y: 0.108237510416
              z: 1.08930171575
              w: 0.0
        
              x: 1.04703892431
      y: -0.462027855191
      z: 0.00357342749709
    orientation: 
      x: 0.00149190984326
      y: 0.0236175934392
      z: -0.671254973314
      w: 0.740848664901

        %}
        
 robot_posn= [ 1.06550891565
      -0.464544294393
      -0.00107756625147 ] 
  robot_quat = Quaternion( [  0.740848664901  0.00149190984326 0.0236175934392 -0.671254973314 ])
  draw_axes_direct(robot_quat.R,robot_posn,'',1)
  
     um = [-1.3956   -0.1179   -0.1983    1.9721
    0.0858   -1.3776    0.2657   -1.7856
   -0.1210    0.2988    0.9632    0.8500
         0         0         0    1.0000]

  draw_axes_direct(um(1:3,1:3),um(1:3,4),'',1)
  
  
     x= 1.98398080149
      y= 0.77873450197
      z= -0.10542449187
    robot_posn=[x;y;z]      
      x= -0.00822159588588
      y= 0.0177235003334
      z= -0.477822976822
      w= 0.878238854593

      robot_quat = Quaternion( [  w   x y z ] )
  draw_axes_direct(robot_quat.R,robot_posn,'',1)
  
  norm_2(robot_posn - camera_posn,1)
  camera_posn - robot_posn

%%  clear up the ROS components
clear('feature_detections_2_sub_w_callback')
clear('node1','node2')  %  clear nodes
clear('m1','m2') % clear masters


%%  




