function camera_extrinsics_data_analysis_copy_data_to_table(points_2D, points_3D, filename)
    global start_idx
    global current_idx
%   [filename]__[start_idx]__[current_idx]__points_[2|3]D.txt
    
    points_2D_part = points_2D(start_idx:current_idx,:);
    points_3D_part = points_3D(start_idx:current_idx,:);
    
    points_2D_part_table = table( ...
        points_2D_part(:,1) , ...
        points_2D_part(:,2) ....
        );
    
    
    points_3D_part_table = table( ...
        points_3D_part(:,1) , ...
        points_3D_part(:,2) , ....
        points_3D_part(:,3) ...
        );
    
    filename_2D_suffix = sprintf('%s__%d__%d__points_%dD.txt', filename, start_idx, current_idx,2)
%     filename_2D = strcat( filename, filename_2D_suffix)
    writetable(points_2D_part_table,filename_2D_suffix)
    filename_3D_suffix = sprintf('%s__%d__%d__points_%dD.txt', filename, start_idx, current_idx,3)
%     filename_3D = strcat( filename, filename_3D_suffix)
    writetable(points_3D_part_table,filename_3D_suffix)
    
    
    start_idx = current_idx;
end

%
%  '/mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/results/2018_02_13_split/2018_02_13_20_56_29/mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/results/2018_02_13_split/2018_02_13_20_56_29__1__5__points_2D.txt'