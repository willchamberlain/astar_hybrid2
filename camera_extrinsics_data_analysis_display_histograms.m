function camera_extrinsics_data_analysis_display_histograms(world_to_camera_trans_hist,world_to_camera_rotm_hist,varargin)
fprintf('\nnargin=%d\n',nargin)
if nargin > 0
    figure(varargin{:})
else
    figure
end
    subplot(2,3,1)
    histogram(world_to_camera_trans_hist(:,1),20)
    title('x')  
    subplot(2,3,2)
    histogram(world_to_camera_trans_hist(:,2),20)
    title('y')  
    subplot(2,3,3)
    histogram(world_to_camera_trans_hist(:,3),20)
    title('z')  
    subplot(2,3,4)
    histogram(world_to_camera_rotm_hist(:,1),20)
    title('qx')  
    subplot(2,3,5)
    histogram(world_to_camera_rotm_hist(:,2),20)
    title('qy')  
    subplot(2,3,6)
    histogram(world_to_camera_rotm_hist(:,3),20)
    title('qz')  

end