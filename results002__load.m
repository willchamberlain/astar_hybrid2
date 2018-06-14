function results002__load(filenamepath_) 
% load the Results_002 object from file <filenamepath_> and assign to workspace variables.

    load(filenamepath_)

    display(results_002__.exp_num)
    display(results_002__.description)
    display('results_002__ : PRESS ANY KEY TO CONTINUE')
    pause
assignin('base','exp_num', results_002__.exp_num) ;
assignin('base','description', results_002__.description) ;
assignin('base','camera', results_002__.camera) ;
assignin('base','true_camera_pose', results_002__.camera.T) ;
assignin('base','qb', results_002__.qb) ;
assignin('base','qbd', results_002__.qbd) ;
assignin('base','qbdd', results_002__.qbdd) ;
assignin('base','start_posn', results_002__.start_posn) ;
assignin('base','via_posns', results_002__.via_posns) ;
assignin('base','axis_speed_limits', results_002__.axis_speed_limits) ;
assignin('base','time_under_acc', results_002__.time_under_acc) ;
assignin('base','time_step', results_002__.time_step) ;
assignin('base','latency_s', results_002__.latency_s) ;
assignin('base','latency_time_steps', results_002__.latency_time_steps) ;
assignin('base','num_points', results_002__.num_points) ;
assignin('base','feature_1_pose_SE3', results_002__.feature_1_pose_SE3) ;
assignin('base','feature_1_positions', results_002__.feature_1_positions) ;
assignin('base','points_3D_f1_indices', results_002__.points_3D_f1_indices) ;
assignin('base','points_3D_f1', results_002__.points_3D_f1) ;
assignin('base','points_3D_f1_latency', results_002__.points_3D_f1_latency) ;
assignin('base','feature_2_pose_SE3', results_002__.feature_2_pose_SE3) ;
assignin('base','feature_2_positions', results_002__.feature_2_positions) ;
assignin('base','points_3D_f2_indices', results_002__.points_3D_f2_indices) ;
assignin('base','points_3D_f2', results_002__.points_3D_f2) ;
assignin('base','points_3D_f2_latency', results_002__.points_3D_f2_latency) ;
assignin('base','feature_3_pose_SE3', results_002__.feature_3_pose_SE3) ;
assignin('base','feature_3_positions', results_002__.feature_3_positions) ;
assignin('base','points_3D_f3_indices', results_002__.points_3D_f3_indices) ;
assignin('base','points_3D_f3', results_002__.points_3D_f3) ;
assignin('base','points_3D_f3_latency', results_002__.points_3D_f3_latency) ;
assignin('base','num_RANSAC_iterations', results_002__.num_RANSAC_iterations) ;
assignin('base','models_extrinsic_estimate_as_local_to_world', results_002__.models_extrinsic_estimate_as_local_to_world) ;
          display(results_002__.exp_num)
          display(results_002__.description)


end