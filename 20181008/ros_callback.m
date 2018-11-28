function ros_callback(src, msg, publisher1, publisher2)
    display('bobb')
    [msg.Position.X;msg.Position.Y;msg.Position.Z]
    display(sprintf('Robot id = %i',floor(msg.Orientation.W)))
    feature_id = floor(msg.Orientation.W)
    W_actual = rem(msg.Orientation.W,1)
    
%     camera_orientation = [
%     0.5676   -0.7956    0.2119
%     0.7453    0.6059    0.2782
%    -0.3497         0    0.9368 ];
%    camera_posn = [
%     1.0000
%    -2.3000
%     1.2200 ];
% camera_pose = rt2tr(camera_orientation,camera_posn)
% 605 at QUT mobile lab 20181015
cam_pose = [
    0.5676   -0.7956    0.2119    1.0000
    0.7453    0.6059    0.2782   -2.3000
   -0.3497         0    0.9368    1.2200
         0         0         0    1.0000 ];
%Nexus 6 at home 20181015_1335
cam_pose = [
   0.684498530415761  -0.563336824641510   0.462723874315433  -1.760000000000000
   0.466703543465291   0.826227342807548   0.315493550669614  -0.880000000000000
  -0.560044252158350                   0   0.828462694165763   1.440000000000000
                   0                   0                   0   1.000000000000000     ];
    
    %  apply the camera pose
    quat = Quaternion( [ W_actual, msg.Orientation.X, msg.Orientation.Y, msg.Orientation.Z] ) 
    tr_SE3 = rt2tr(quat.R,[msg.Position.X;msg.Position.Y;msg.Position.Z]);
    tr_world_to_base_link = cam_pose*tr_SE3 
    [ tr_world_to_base_link_rot , tr_world_to_base_link_trans ] = tr2rt(tr_world_to_base_link) 
    world_to_base_link_quat = Quaternion(tr_world_to_base_link_rot)
    
    
    publisher__ = []
    if feature_id == 170
        publisher__ = publisher2
    else
        publisher__ = publisher1
    end
    odometry_ = rosmessage(publisher__)
    odometry_.Header.FrameId='map';
    
    odometry_.Pose.Pose.Position.X = msg.Position.X;
    odometry_.Pose.Pose.Position.Y = msg.Position.Y;
    odometry_.Pose.Pose.Position.Z = msg.Position.Z;
    
    odometry_.Pose.Pose.Orientation.X = world_to_base_link_quat.v(1); % msg.Orientation.X;
    odometry_.Pose.Pose.Orientation.Y = world_to_base_link_quat.v(2); % msg.Orientation.Y;
    odometry_.Pose.Pose.Orientation.Z = world_to_base_link_quat.v(3); % msg.Orientation.Z;
    odometry_.Pose.Pose.Orientation.W = world_to_base_link_quat.s; % W_actual;
    
    send(publisher__,odometry_)
end