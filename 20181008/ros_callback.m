function ros_callback(src, msg, publisher1, publisher2)
    display('bobb')
    [msg.Position.X;msg.Position.Y;msg.Position.Z]
    display(sprintf('Robot id = %i',floor(msg.Orientation.W)))
    feature_id = floor(msg.Orientation.W)
    W_actual = rem(msg.Orientation.W,1)
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
    
    odometry_.Pose.Pose.Orientation.X = msg.Orientation.X;
    odometry_.Pose.Pose.Orientation.Y = msg.Orientation.Y;
    odometry_.Pose.Pose.Orientation.Z = msg.Orientation.Z;
    odometry_.Pose.Pose.Orientation.W = W_actual;
    
    send(publisher__,odometry_)
end