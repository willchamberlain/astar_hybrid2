
    %camera_target_point = [ 1.5 1.2 0.645 1]'  ;
    target_point_indices = randperm(num_datapoints,2)
    camera_target_point = mean(points_3D_random_hom(:,target_point_indices),2)
    
    camera_position = [5 3 2 1]'  ;
    camera_target_point - camera_position;
    opp = camera_target_point(3) - camera_position(3)
    adj_horz = sqrt(  sum(  (camera_target_point(1:2)-camera_position(1:2)).^2  , 1)  )
    pitch_down_rads = atan(opp/adj_horz)
    pitch_down_rads = atan2(opp,adj_horz)
    pitch_down_degs = radtodeg(pitch_down_rads)    
%             figure;  plot( [ 0 0 3.935733730830886 0 ] , [ 0 1.35 1.35  0 ]  ) ; axis equal ; grid on; hold on;
%             plot( [ 0 0   3.935733730830886   0 ] , [ 0   3.935733730830886     3.935733730830886    0 ]  ) ; 
%             plot( [ 0 0   3.935733730830886   0 ] , [ 0   3.935733730830886/2   3.935733730830886/2  0 ]  ) ; 
    forward = camera_target_point(1) - camera_position(1)
    left =  camera_target_point(2) - camera_position(2)
    yaw_left_rads = atan(left/forward)
    yaw_left_rads = atan2(left,forward)
    yaw_left_degs = radtodeg(yaw_left_rads)
%             figure;  plot( [ 0 0 -left 0 ] , [ 0 forward forward  0 ]  ) ; axis equal ; grid on; hold on;
%             plot( [ 0 0 -left 0 ] , [ 0 left  left  0 ]  ) ;    
%             plot( [ 0 0 -left 0 ] , [ 0 left/2  left/2  0 ]  ) ; 
    camera_pose = [ [ rotz(yaw_left_rads) * roty(pitch_down_rads) * rotz(0)  ; 0 0 0 ] , [ 0 0 0 1 ]' ] *  [  [ rotz(0) * roty(0) * rotz(0) ; 0 0 0 ] , camera_position ]
    camera_pose = [ [ rotz(yaw_left_rads) ; 0 0 0 ] , camera_position ]
    camera_pose = [ [ roty(pitch_down_rads) ; 0 0 0 ] , [ 0 0 0 1 ]' ]    * camera_pose
%     camera_pose = [ [ rotz(yaw_left_rads) * roty(pitch_down_rads) * rotz(0)  ; 0 0 0 ] , camera_position ]
%     camera_pose = [ [ rotz(0) * roty(0) * rotz(yaw_left_rads)  ; 0 0 0 ] , camera_position ]
    figure('Name','plot camera'); hold on; xlabel('x'); ylabel('y'); zlabel('z')'; hold on;
    plot3_rows(camera_position,'bs'); plot3_rows(camera_target_point,'rx');
    axis equal ; grid on; hold on;
    
    %suspect 
    draw_axes_direct_c(geom__transform_to_rotation_SO3(camera_pose), geom__transform_to_translation(camera_pose), 'a',adj_horz,'m');
    draw_axes_direct(geom__transform_to_rotation_SO3(camera_pose), geom__transform_to_translation(camera_pose), 'a',0.75*adj_horz);
%     end_pt = camera_pose(1:3,1:3)* ( eye(3) * adj )
%     plot3_rows( end_pt(:,1) , 'ks')

    % yaw by itself is OK
    camera_pose = [ [ rotz(yaw_left_rads) ; 0 0 0 ] , camera_position ]  % = rt2tr(rotz(yaw_left_rads),h2e(camera_position))
    end_pt_2 = camera_pose *  geo__euclidean_3D_to_hom(adj_horz*eye(3))
    plot3_rows( end_pt_2(:,1) , 'bd', 'Linewidth',3)
    draw_axes_direct_c(camera_pose(1:3,1:3), camera_pose(1:3,4), {'cam_position not rot','yaw only','yaw only','yaw only'},adj_horz,'c');
    draw_axes_direct(camera_pose(1:3,1:3), camera_pose(1:3,4), 'cam_position not rot', 0.75*adj_horz);
    
    % pitch by itself is OK
    camera_pose = [ [ roty(pitch_down_rads*-1) ; 0 0 0 ] , camera_position ]  % = rt2tr(rotz(yaw_left_rads),h2e(camera_position))
%     end_pt_3 = rotm2tform(roty(pitch_down_rads)) * (camera_pose  *  geo__euclidean_3D_to_hom(adj*eye(3)) )
    end_pt_3 = camera_pose *  geo__euclidean_3D_to_hom(adj_horz*eye(3))     
    plot3_rows( end_pt_3(:,1) , 'gd', 'Linewidth',3)
    draw_axes_direct_c(camera_pose(1:3,1:3), camera_pose(1:3,4), {'cam_position not rot','pitch','pitch','pitch'},adj_horz,'c');
    draw_axes_direct(camera_pose(1:3,1:3), camera_pose(1:3,4), 'cam_position not rot', 0.75*adj_horz);
    
%     end_pt_pitch_only_transform = rt2tr(roty(pitch_down_rads),h2e(camera_position))
%     end_pt_pitch_only = end_pt_pitch_only_temp * geo__euclidean_3D_to_hom(adj*eye(3))
%     plot3_rows( end_pt_pitch_only(:,1) , 'mo', 'Linewidth',1)   % check the angle up
    
    % yaw then pitch:  problems 
    opp = camera_target_point(3) - camera_position(3)
    hyp_xyz = norm((camera_target_point-camera_position).^2  , 2) 
    adj_horz = sqrt(  sum(  (camera_target_point(1:2)-camera_position(1:2)).^2  , 1)  )
    pitch_down_rads = atan(opp/adj_horz)
    pitch_down_rads = atan2(opp,hyp_xyz)
    pitch_down_degs = radtodeg(pitch_down_rads)    
     camera_pose = [ [ roty(pitch_down_rads) * rotz(yaw_left_rads) ; 0 0 0 ] , camera_position ] 
     
    pitch_down_rads =  acos(adj/hyp_xyz)
    pitch_down_degs = radtodeg(pitch_down_rads)  
     camera_pose = [ [ roty(pitch_down_rads) * rotz(yaw_left_rads) ; 0 0 0 ] , camera_position ] 
     end_pt_yaw_then_pitch = camera_pose *  geo__euclidean_3D_to_hom(adj_horz*eye(3))    
    plot3_rows( end_pt_3(:,1) , 'md', 'Linewidth',3)
    draw_axes_direct_c(camera_pose(1:3,1:3), camera_pose(1:3,4), {'cam_position not rot','yaw pitch','yaw pitch','yaw pitch'},adj_horz,'c');
    draw_axes_direct(camera_pose(1:3,1:3), camera_pose(1:3,4), 'cam_position not rot', 0.75*adj_horz);
    