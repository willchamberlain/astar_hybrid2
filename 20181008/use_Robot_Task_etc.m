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
    pause(3)
    rosinit
    pause(2)
    
    ros_tf_tree = rostf
    pause(1)
    
   %%
    
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
    
    
    pioneer1_base_pose_ground_truth_publisher = rospublisher('/base_pose_ground_truth_matlab', 'nav_msgs/Odometry')  ;
    pioneer1_base_pose_ground_truth_msg = rosmessage(pioneer1_base_pose_ground_truth_publisher)  ;
    
    while true  %  NOTE:  needs a few iterations for tf to start making the values available (?why?)
    for robot_ii_ = 1:size(robots,2)
        %display('Robots:')
        robot_ = robots(robot_ii_)  ;
        for tag_ii_ = 1:size(robot_.tag_topics,2)           
            %display('Tags:')
            tag_topic = robot_.tag_topics{tag_ii_} ;
            should_send = false;
            try 
                waitForTransform(ros_tf_tree, 'map', tag_topic, 0.5);
                map_to_robot_tf = getTransform(ros_tf_tree, 'map', tag_topic)  ;
                
                now_time = rostime('Now')  ;
                if now_time.Sec - map_to_robot_tf.Header.Stamp.Sec > 0 
                    %display('TOO OLD  SECONDS  - skipping')
                else
                    if now_time.Nsec - map_to_robot_tf.Header.Stamp.Nsec > 500000000
                        %display('TOO OLD  NANOseconds - skipping')                        

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
                            running_interp = Quaternion([quat.W, quat.X,quat.Y,quat.Z]) ;   % have to initialise to _something_ .
                            %{
                            for poseii_ = 1: size(robot_poses{robot_ii_},2)
                                x_(poseii_) = robot_poses{robot_ii_}{poseii_}.Transform.Rotation.X
                                y_(poseii_) = robot_poses{robot_ii_}{poseii_}.Transform.Rotation.Y
                                z_(poseii_) = robot_poses{robot_ii_}{poseii_}.Transform.Rotation.Z
                                w_(poseii_) = robot_poses{robot_ii_}{poseii_}.Transform.Rotation.W
                            end
                            figure; hold on; grid on; plot(x_); plot(y_); plot(z_); plot(w_); 
                            %}
                            for poseii_ = 1: size(robot_poses{robot_ii_},2)

                                if now_time.Sec -  robot_poses{robot_ii_}{poseii_}.Header.Stamp.Sec > 0 
                                    %display('TOO OLD  SECONDS  - skipping')
                                else
                                    if now_time.Nsec - robot_poses{robot_ii_}{poseii_}.Header.Stamp.Nsec > 500000000
                                        %display('TOO OLD  NANOseconds - skipping')
                                    else
                                        should_send = true ;
                                        display('GOOD FRESH DATA')
                                         transl = robot_poses{robot_ii_}{poseii_}.Transform.Translation ;
                                        sum_transl = sum_transl + [transl.X;transl.Y;transl.Z] ;
                                        quat = robot_poses{robot_ii_}{poseii_}.Transform.Rotation ;
                                        q = Quaternion([quat.W, quat.X,quat.Y,quat.Z]) ;  % <s , [v1 v2 v3] >
                                        running_interp = q.interp(running_interp,0.5) ;     %  should be a weighted interpolation
                                    end
                                end

                            end
                            mean_transl = sum_transl./size(robot_poses{robot_ii_},2) ;

                                        should_send = true ;
                                        
                            if 1 == robot_ii_  &&  should_send
                                display('should_send')
                                display('should_send')
                                display('should_send')
                                display('should_send')
                                covariance = ...
                                    [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, ...
                                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0, ...
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...
                                    0.0, 0.0, 0.0, 0.0, 0.0, 1.0]'  ;
                                pioneer1_base_pose_ground_truth_msg.Header.FrameId = 'map'  ;
                                pioneer1_base_pose_ground_truth_msg.Header.Stamp = rostime('now')  ;
                                % pioneer1_base_pose_ground_truth_msg.Pose.Pose.Position = mean_transl  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Position.X = mean_transl(1)  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Position.Y = mean_transl(2)  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Position.Z = mean_transl(3)  ;
                                % pioneer1_base_pose_ground_truth_msg.Pose.Pose.Orientation = running_interp  ; 
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Orientation.W = running_interp.s  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Orientation.X = running_interp.v(1)  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Orientation.Y = running_interp.v(2)  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Orientation.Z = running_interp.v(3)  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Covariance = covariance  ;
                                pioneer1_base_pose_ground_truth_msg.Twist.Covariance = covariance  ;                        
                                pioneer1_base_pose_ground_truth_msg.Twist.Twist.Linear.X = 0  ;
                                pioneer1_base_pose_ground_truth_msg.Twist.Twist.Linear.Y = 0  ;
                                pioneer1_base_pose_ground_truth_msg.Twist.Twist.Linear.Z = 0  ;
                                pioneer1_base_pose_ground_truth_msg.Twist.Twist.Angular.X = 0  ;
                                pioneer1_base_pose_ground_truth_msg.Twist.Twist.Angular.Y = 0  ;
                                pioneer1_base_pose_ground_truth_msg.Twist.Twist.Angular.Z = 0  ;                        
                                pioneer1_base_pose_ground_truth_msg
                                pioneer1_base_pose_ground_truth_msg.Header.FrameId = strrep(tag_topic,'_b','') ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Position =  map_to_robot_tf.Transform.Translation  ;
                                pioneer1_base_pose_ground_truth_msg.Pose.Pose.Orientation =  map_to_robot_tf.Transform.Rotation  ;
                                pioneer1_base_pose_ground_truth_msg
                                pioneer1_base_pose_ground_truth_publisher.send(pioneer1_base_pose_ground_truth_msg);
                            end

                        else
                            robot_pose{end+1} = map_to_robot_tf  ;
                            robot_poses{robot_ii_} = robot_pose  ;
                        end
                    end
                end
            catch e %e is an MException struct     %   see  https://au.mathworks.com/matlabcentral/answers/325475-display-error-message-and-execute-catch
        fprintf(1,'The identifier was:\n%s',e.identifier);
        fprintf(1,'There was an error! The message was:\n%s',e.message);
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


