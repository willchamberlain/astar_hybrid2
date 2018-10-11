%  see https://au.mathworks.com/help/robotics/examples/connect-to-a-ros-network.html
%       https://au.mathworks.com/help/robotics/examples/exchange-data-with-ros-publishers.html

addpath('/mnt/nixbig/ownCloud/project_code/')
addpath('/mnt/nixbig/ownCloud/project_code/20181008/')

%%

rosshutdown


%%

getenv('ROS_MASTER_URI')
getenv('ROS_HOSTNAME')
getenv('ROS_IP')

setenv('ROS_MASTER_URI','http://172.19.30.206:11311')
setenv('ROS_IP','172.19.30.206')
setenv('ROS_HOSTNAME','172.19.30.206')


setenv('ROS_MASTER_URI','http://192.168.86.202:11311')
setenv('ROS_IP','192.168.86.202')
setenv('ROS_HOSTNAME','192.168.86.202')


setenv('ROS_MASTER_URI','http://192.168.86.202:11311')
setenv('ROS_IP','192.168.86.202')
setenv('ROS_HOSTNAME','192.168.86.202')


rosinit

%%
% check - can connect to ROS

rosnode list
rostopic list
rosservice list

tf_tree = rostf

mumblemumble_pub = rospublisher('/mumblemumble', 'std_msgs/String')
mumblemumble_msg = rosmessage(mumblemumble_pub)
mumblemumble_msg.Data='mumblemumble'
mumblemumble_pub.send(mumblemumble_msg)

%%
% get plain message for   cmd_vel   topic to use via  send_cmd_vel_planar_safer

pioneer2_cmd_vel_pub = rospublisher('/Pioneer2/cmd_vel', 'geometry_msgs/Twist')

pioneer2_cmd_vel_msg = rosmessage(pioneer2_cmd_vel_pub)


% vel = 1.0
%  ~0.85 with roll , ~0.8 under power
send_cmd_vel_planar_safer(  1.0 , 0.0,  2 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%  mostly symmetric
send_cmd_vel_planar_safer(  -1.0 , 0.0,  2 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
% ~4.8m
send_cmd_vel_planar_safer(  1.0 , 0.0,  10 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
send_cmd_vel_planar_safer(  -1.0 , 0.0,  10 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)

%%
%4m - 4.2m
send_cmd_vel_planar_safer(  1.0 , 0.0,  8.3 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%
% ~2.5m
send_cmd_vel_planar_safer(  1.0 , 0.0,  8.3*2.5/4 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%
%~1.5m
send_cmd_vel_planar_safer(  1.0 , 0.0,  8.3*1.5/4 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)

%%
% forward  10cm NOTE: deal with the castor inducing a rotation
send_cmd_vel_planar_safer(  0.1 , 0.0,  1 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%
% backward 10cm NOTE: deal with the castor inducing a rotation
send_cmd_vel_planar_safer(  -0.1 , 0.0,  1 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)

%%
% anti-clockwise / left - rate of turn = 0.2
send_cmd_vel_planar_safer(  0 , 0.2,  0.2 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%
% clockwise / right - rate of turn = 0.2
send_cmd_vel_planar_safer(  0 , -0.2,  0.2 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%

%%
% rate of turn = 0.5
% one half turn
send_cmd_vel_planar_safer(  0 , 0.5,  6.4 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%
% rate of turn = 1.0
% one turn - note ~0.05 to 0.1 overshoot with extra 0.5 rotational velocity
send_cmd_vel_planar_safer(  0 , 1.0,  6.3 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%
% one half turn
send_cmd_vel_planar_safer(  0 , 1.0,  3.15 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)
%%



send_cmd_vel_planar_safer(  0 , -0.2,  1 , pioneer2_cmd_vel_pub , pioneer2_cmd_vel_msg)


%%

pioneer2_cmd_vel_pub = rospublisher('/Pioneer2/cmd_vel', 'geometry_msgs/Twist')

left_pioneer2_cmd_vel_rmsg = rosmessage(pioneer2_cmd_vel_pub);  left_pioneer2_cmd_vel_rmsg.showdetails  ;
left_pioneer2_cmd_vel_rmsg.Linear.X =  0 ; left_pioneer2_cmd_vel_rmsg.Angular.Z = 0.5;

stop_pioneer2_cmd_vel_rmsg = rosmessage(pioneer2_cmd_vel_pub);
stop_pioneer2_cmd_vel_rmsg.Linear.X =  0.0; stop_pioneer2_cmd_vel_rmsg.Angular.Z = 0.0  ;

%  One full turn  -  approx  -  needs calibrating and closing the loop 
pioneer2_cmd_vel_pub.send(left_pioneer2_cmd_vel_rmsg)
pause(12.4)
pioneer2_cmd_vel_pub.send(stop_pioneer2_cmd_vel_rmsg)

send_cmd_vel(5,0,0,0,0,25,1,pioneer2_cmd_vel_pub,left_pioneer2_cmd_vel_rmsg)

pioneer2_cmd_vel_pub.send(left_pioneer2_cmd_vel_rmsg)
pause(12.4)
pioneer2_cmd_vel_pub.send(stop_pioneer2_cmd_vel_rmsg)



pioneer2_cmd_vel_pub.send(left_pioneer2_cmd_vel_rmsg)


spiral_pioneer2_cmd_vel_rmsg = rosmessage(pioneer2_cmd_vel_pub);
spiral_pioneer2_cmd_vel_rmsg.Linear.X =  0.1
spiral_pioneer2_cmd_vel_rmsg.Angular.Z = 0.05

spiralb_pioneer2_cmd_vel_rmsg = rosmessage(pioneer2_cmd_vel_pub);
spiralb_pioneer2_cmd_vel_rmsg.Linear.X =  -0.1
spiralb_pioneer2_cmd_vel_rmsg.Angular.Z = -0.05

pioneer2_cmd_vel_pub.send(stop_pioneer2_cmd_vel_rmsg)




pioneer1_cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist')

left_pioneer1_cmd_vel_rmsg = rosmessage(pioneer1_cmd_vel_pub);
left_pioneer1_cmd_vel_rmsg.Linear.X =  0.0
left_pioneer1_cmd_vel_rmsg.Angular.Z = 0.5


right_pioneer1_cmd_vel_rmsg = rosmessage(pioneer1_cmd_vel_pub);
right_pioneer1_cmd_vel_rmsg.Linear.X =  0.0
right_pioneer1_cmd_vel_rmsg.Angular.Z = -0.5


forward_pioneer1_cmd_vel_rmsg = rosmessage(pioneer1_cmd_vel_pub);
forward_pioneer1_cmd_vel_rmsg.Linear.X =  0.1
forward_pioneer1_cmd_vel_rmsg.Angular.Z = 0.0


spiral_pioneer1_cmd_vel_rmsg = rosmessage(pioneer1_cmd_vel_pub);
spiral_pioneer1_cmd_vel_rmsg.Linear.X =  0.1
spiral_pioneer1_cmd_vel_rmsg.Angular.Z = 0.05

spiralb_pioneer1_cmd_vel_rmsg = rosmessage(pioneer1_cmd_vel_pub);
spiralb_pioneer1_cmd_vel_rmsg.Linear.X =  -0.1
spiralb_pioneer1_cmd_vel_rmsg.Angular.Z = -0.05

stop_pioneer1_cmd_vel_rmsg = rosmessage(pioneer1_cmd_vel_pub);
stop_pioneer1_cmd_vel_rmsg.Linear.X =  0.0 ; stop_pioneer1_cmd_vel_rmsg.Angular.Z = 0.0 ;

%turn 90 left
pioneer1_cmd_vel_pub.send(left_pioneer1_cmd_vel_rmsg)
pause(3)
pioneer1_cmd_vel_pub.send(stop_pioneer1_cmd_vel_rmsg)


pioneer1_cmd_vel_pub.send(right_pioneer1_cmd_vel_rmsg)
pause(3)
pioneer1_cmd_vel_pub.send(stop_pioneer1_cmd_vel_rmsg)


pioneer1_cmd_vel_pub.send(left_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(forward_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(right_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(forward_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(left_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(forward_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(right_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(forward_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(stop_pioneer1_cmd_vel_rmsg)

%%



pioneer1_cmd_vel_pub.send(forward_pioneer1_cmd_vel_rmsg)
pause(20)
pioneer1_cmd_vel_pub.send(stop_pioneer1_cmd_vel_rmsg)

%%

pioneer1_cmd_vel_pub.send(right_pioneer1_cmd_vel_rmsg)
pause(2)
pioneer1_cmd_vel_pub.send(spiral_pioneer1_cmd_vel_rmsg)
pause(40)
pioneer1_cmd_vel_pub.send(stop_pioneer1_cmd_vel_rmsg)

pioneer1_cmd_vel_pub.send(spiralb_pioneer1_cmd_vel_rmsg)
pause(40)
pioneer1_cmd_vel_pub.send(stop_pioneer1_cmd_vel_rmsg)


pioneer2_cmd_vel_pub.send(spiralb_pioneer2_cmd_vel_rmsg)
pause(40)
pioneer2_cmd_vel_pub.send(stop_pioneer2_cmd_vel_rmsg)


