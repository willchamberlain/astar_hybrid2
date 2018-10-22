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


setenv('ROS_MASTER_URI','http://192.168.86.202:11311')
setenv('ROS_IP','192.168.86.202')
setenv('ROS_HOSTNAME','192.168.86.202')


rosinit


%%


rosnode list
rostopic list
rosservice list

tf_tree = rostf
