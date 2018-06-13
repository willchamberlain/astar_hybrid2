function results001__load(filenamepath_) 
% load the Results_001 object from file <filenamepath_> and assign to workspace variables.

    load(filenamepath_)

    display(results_001__.exp_num)
    display(results_001__.description)
    display('PRESS ANY KEY TO CONTINUE')
    pause
assignin('base','exp_num', results_001__.exp_num) ;
assignin('base','description', results_001__.description) ;
assignin('base','camera', results_001__.camera) ;
assignin('base','true_camera_pose', results_001__.camera.T) ;
assignin('base','qb', results_001__.qb) ;
assignin('base','qbd', results_001__.qbd) ;
assignin('base','qbdd', results_001__.qbdd) ;
assignin('base','start_posn', results_001__.start_posn) ;
assignin('base','via_posns', results_001__.via_posns) ;
assignin('base','axis_speed_limits', results_001__.axis_speed_limits) ;
assignin('base','time_under_acc', results_001__.time_under_acc) ;
assignin('base','time_step', results_001__.time_step) ;
assignin('base','latency_s', results_001__.latency_s) ;
assignin('base','latency_time_steps', results_001__.latency_time_steps) ;
assignin('base','num_points', results_001__.num_points) ;
assignin('base','feature_1_pose_SE3', results_001__.feature_1_pose_SE3) ;
assignin('base','feature_1_positions', results_001__.feature_1_positions) ;
assignin('base','points_3D_f1_indices', results_001__.points_3D_f1_indices) ;
assignin('base','points_3D_f1', results_001__.points_3D_f1) ;
assignin('base','points_3D_f1_latency', results_001__.points_3D_f1_latency) ;
assignin('base','feature_2_pose_SE3', results_001__.feature_2_pose_SE3) ;
assignin('base','feature_2_positions', results_001__.feature_2_positions) ;
assignin('base','points_3D_f2_indices', results_001__.points_3D_f2_indices) ;
assignin('base','points_3D_f2', results_001__.points_3D_f2) ;
assignin('base','points_3D_f2_latency', results_001__.points_3D_f2_latency) ;
assignin('base','num_RANSAC_iterations', results_001__.num_RANSAC_iterations) ;
assignin('base','models_extrinsic_estimate_as_local_to_world', results_001__.models_extrinsic_estimate_as_local_to_world) ;
          display(results_001__.exp_num)
          display(results_001__.description)


end