% start setup

pioneer1 = Robot([250,290,330])
pioneer1_task = Task( [170] )
pioneer1.task = pioneer1_task



pioneer2 = Robot([170])
pioneer2_task = Task( [250,290,330] )
pioneer2.task = pioneer2_task
    %
    ros_tf_tree = rostf
    
    %

% end setup


% stub properties
    vos_client = VOS_Client
% end stub properties



%  Stub out a RobotController here:  pioneer2.robotController , so can drop the 'pioneer2.' from below once this is a class

    %   make request for one-off localisation
    if ~pioneer1.is_localised()
        % make a request to VOS
        display('Need to localise')
        vos_client.where_is(pioneer1.robot_visual_model, robot.pose_most_likely, robot.pose_uncertainty)
    end
    
    %   make request for one-off localisation
    if ~pioneer1.task.is_target_pose_known()
        % make a request to VOS
        display('Need to find the target')      
        vos_client.where_is(pioneer1.task.target_model, robot.target_pose_most_likely, robot.target_pose_uncertainty)
    end
    
    %---------
    
    %%
    rosshutdown
    pause(1)
    rosinit
    pause(1)
    
    ros_tf_tree = rostf
    pause(1)
    
%     robot_guiabot.name='Guiabot'
%     robot_guiabot.tag_topics{1}='dum_c603_trans_rot_to_t50090_actual_b'
%     robot_guiabot.tag_topics{2}='dum_c603_trans_rot_to_t50130_actual_b'
%     robot_guiabot.tag_topics{3}='dum_c603_trans_rot_to_t50170_actual_b'
%     robot_guiabot.tag_topics{4}='dum_c603_trans_rot_to_t50210_actual_b'
    
    robot_pioneer1.name='Pioneer1'
    robot_pioneer1.tag_topics{1}='dum_c603_trans_rot_to_t70250_actual_b'
    robot_pioneer1.tag_topics{2}='dum_c603_trans_rot_to_t70290_actual_b'
    robot_pioneer1.tag_topics{3}='dum_c603_trans_rot_to_t70330_actual_b'
    
    robot_pioneer2.name='Pioneer2'
    robot_pioneer2.tag_topics{1}='dum_c603_trans_rot_to_t70170_actual_b'
    
%     robots = [ robot_guiabot , robot_pioneer1 ]
    robots = [ robot_pioneer1 , robot_pioneer2 ]  
    
    robot_poses{1} = {} ; 
    robot_poses{2} = {} ; 
    
    while true  %  NOTE:  needs a few iterations for tf to start making the values available (?why?)
    for robot_ii_ = 1:size(robots,2)
        display('Robots:')
        robot_ = robots(robot_ii_)  
        for tag_ii_ = 1:size(robot_.tag_topics,2)           
            display('Tags:')
            tag_topic = robot_.tag_topics{tag_ii_} 
            try 
                waitForTransform(ros_tf_tree, 'map', tag_topic, 0.2);
                map_to_robot_tf = getTransform(ros_tf_tree, 'map', tag_topic)  ;
                map_to_robot_tf.ChildFrameId  
                map_to_robot_tf.Header.FrameId  
                map_to_robot_tf.Transform.Translation  ;     % Vector3 
                map_to_robot_tf.Transform.Rotation  ;          % Quaternion
                display(sprintf('GOOD:  found robot %s tag %s',robot_.name,tag_topic))
                robot_pose = robot_poses{robot_ii_}  ;
                filter_length = 10 ;
                if size(  robot_pose,2) > filter_length
                    for rpii_ = 1:min(size(  robot_pose,2)-1, filter_length)
                        robot_pose{rpii_} = robot_pose{rpii_+1} ;
                    end
                    robot_pose{end} = map_to_robot_tf  ;
                    robot_poses{robot_ii_} = robot_pose  ;
                    
                    % average - run the filtering 
                    sum_transl = zeros(3,1) ;
                    quat = robot_poses{robot_ii_}{poseii_}.Transform.Rotation ;   % have to initialise to _something_ .
                    running_interp = Quaternion(quat.W, [quat.X,quat.Y,quat.Z]) ;   % have to initialise to _something_ .
                    for poseii_ = 1: size(robot_poses{robot_ii_},2)
                        transl = robot_poses{robot_ii_}{poseii_}.Transform.Translation ;
                        sum_transl = sum_transl + [transl.X;transl.Y;transl.Z] ;
                        quat = robot_poses{robot_ii_}{poseii_}.Transform.Rotation ;
                        q = Quaternion(quat.W, [quat.X,quat.Y,quat.Z]) ;  % <s , [v1 v2 v3] >
                        running_interp = q.interp(running_interp,0.5) ;     %  should be a weighted interpolation
                    end
                    mean_transl = sum_transl./size(robot_poses{robot_ii_},2) ;
                    running_interp
                else
                    robot_pose{end+1} = map_to_robot_tf  ;
                    robot_poses{robot_ii_} = robot_pose  ;
                end
            catch
                display(sprintf('problem with robot %s tag %s',robot_.name,tag_topic))
            end             
        end
    end
    end
    
    %        % Wait for the transform that will take data from
    %        % 'camera_depth_frame' to 'base_link'. This will be blocking until 
    %        % the transformation is valid.
    %        waitForTransform(tftree, 'base_link', 'camera_depth_frame');
    %
    %        % Define a point [3 1.5 0.2] in the camera's coordinate frame
    %        pt = rosmessage('geometry_msgs/PointStamped');
    %        pt.Header.FrameId = 'camera_depth_frame';
    %        pt.Point.X = 3;
    %        pt.Point.Y = 1.5;
    %        pt.Point.Z = 0.2;
    %
    %        % Transformation is available, so transform the point into the /base_link frame
    %        tfpt = transform(tftree, 'base_link', pt)
    %
    %        % Display the transformed point coordinates
    %        tfpt.Point
    %
    %   See also robotics.ros.TransformationTree


