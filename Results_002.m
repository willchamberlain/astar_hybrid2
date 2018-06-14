classdef Results_002 
% container for interesting workspace variables which can easily be saved to a file: version 1	
    properties
        exp_num
        description
        camera
        true_camera_pose
        qb
        qbd
        qbdd            
        start_posn
        via_posns 
        axis_speed_limits
        time_under_acc
        time_step
        latency_s
        latency_time_steps
        num_points
        feature_1_pose_SE3
        feature_1_positions
        points_3D_f1_indices
        points_3D_f1 
        points_3D_f1_latency
        feature_2_pose_SE3
        feature_2_positions
        points_3D_f2_indices
        points_3D_f2
        points_3D_f2_latency 
        feature_3_pose_SE3
        feature_3_positions
        points_3D_f3_indices
        points_3D_f3
        points_3D_f3_latency 
        num_RANSAC_iterations
        models_extrinsic_estimate_as_local_to_world
    end
    methods
        function obj = Results_002(exp_num_, description_ ,  camera_ ,  qb_ ,  qbd_ ,  qbdd_ ,  start_posn_ ,  via_posns_ ,  axis_speed_limits_ ,  time_under_acc_ ,  time_step_ ,  latency_s_ ,  latency_time_steps_ ,  num_points_ ,  ...
                feature_1_pose_SE3_ ,  feature_1_positions_ ,  points_3D_f1_indices_ ,  points_3D_f1_ ,  points_3D_f1_latency_ ,  ...
                feature_2_pose_SE3_ ,  feature_2_positions_ ,  points_3D_f2_indices_ ,  points_3D_f2_ ,  points_3D_f2_latency_,  ...
                feature_3_pose_SE3_ ,  feature_3_positions_ ,  points_3D_f3_indices_ ,  points_3D_f3_ ,  points_3D_f3_latency_ ,  ...
                num_RANSAC_iterations_ ,  models_extrinsic_estimate_as_local_to_world_ )
            if nargin > 0
                obj.exp_num = exp_num_;
                obj.description=description_  ;
                obj.camera=camera_  ;
                obj.true_camera_pose=obj.camera.T  ;
                obj.qb=qb_  ;
                obj.qbd=qbd_  ;
                obj.qbdd    =qbdd_  ;
                obj.start_posn=start_posn_  ;
                obj.via_posns =via_posns_  ;
                obj.axis_speed_limits=axis_speed_limits_  ;
                obj.time_under_acc=time_under_acc_  ;
                obj.time_step=time_step_  ;
                obj.latency_s=latency_s_  ;
                obj.latency_time_steps=latency_time_steps_  ;
                obj.num_points=num_points_  ;
                obj.feature_1_pose_SE3=feature_1_pose_SE3_  ;
                obj.feature_1_positions=feature_1_positions_  ;
                obj.points_3D_f1_indices=points_3D_f1_indices_  ;
                obj.points_3D_f1 =points_3D_f1_  ;
                obj.points_3D_f1_latency=points_3D_f1_latency_  ;
                obj.feature_2_pose_SE3=feature_2_pose_SE3_  ;
                obj.feature_2_positions=feature_2_positions_  ;
                obj.points_3D_f2_indices=points_3D_f2_indices_  ;
                obj.points_3D_f2=points_3D_f2_  ;
                obj.points_3D_f2_latency =points_3D_f2_latency_  ;
                obj.feature_3_pose_SE3=feature_3_pose_SE3_  ;
                obj.feature_3_positions=feature_3_positions_  ;
                obj.points_3D_f3_indices=points_3D_f3_indices_  ;
                obj.points_3D_f3=points_3D_f3_  ;
                obj.points_3D_f3_latency =points_3D_f3_latency_  ;
                obj.num_RANSAC_iterations=num_RANSAC_iterations_  ;
                obj.models_extrinsic_estimate_as_local_to_world=models_extrinsic_estimate_as_local_to_world_  ;
            end
        end
    end
end