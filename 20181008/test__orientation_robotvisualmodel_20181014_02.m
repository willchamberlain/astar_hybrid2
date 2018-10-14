
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

setenv('ROS_MASTER_URI','http://192.168.86.202:11311')
setenv('ROS_IP','192.168.86.202')
setenv('ROS_HOSTNAME','192.168.86.202')


rosinit()  % global node, and try to connect to localhost:11311

%  start ROS Masters / roscores  -  see  https://au.mathworks.com/matlabcentral/answers/353299-does-the-robotics-system-toolbox-support-connecting-two-ros-master-core-with-different-ports
m1 = robotics.ros.Core; % Default port of 11311
m2 = robotics.ros.Core(12000);

% start a node
node1 = robotics.ros.Node('/test_node_1','192.168.86.202',11311)
% attach a subscriber to the node
feature_detections_2_sub = robotics.ros.Subscriber(node1, '/feature_detections_2') 
%  feature_detections_2_sub.addlistener
feature_detections_2_sub_w_callback = robotics.ros.Subscriber(node1, '/feature_detections',@{ros_callback,}) 


feature_detections_2_sub_w_callback

% start another node - can be on a different master / URI 
node2 = robotics.ros.Node('/test_node_2','localhost',12000);


% fake two more masters and check communications
master_pioneer1 = robotics.ros.Core(12100); 
master_pioneer2 = robotics.ros.Core(12200); 

clear('pub_pioneer2')
clear('node_pioneer2')
%  node_pioneer2 = robotics.ros.Node('/node_pioneer2__base_pose_ground_truth','192.168.86.29',11311)
node_pioneer2 = robotics.ros.Node('/node_pioneer2__base_pose_ground_truth','192.168.86.202',12200)
pub_pioneer2 = robotics.ros.Publisher(node_pioneer2,'/base_pose_ground_truth','nav_msgs/Odometry')
[node_pioneer2.MasterURI node_pioneer2.NodeURI] 


clear('pub_pioneer1')
clear('node_pioneer1')
%  node_pioneer1 = robotics.ros.Node('/node_pioneer1__base_pose_ground_truth','192.168.86.30',11311)
node_pioneer1 = robotics.ros.Node('/node_pioneer1__base_pose_ground_truth','192.168.86.202',12100)
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

%%  clear up the ROS components
clear('feature_detections_2_sub_w_callback')
clear('node1','node2')  %  clear nodes
clear('m1','m2') % clear masters
%%


