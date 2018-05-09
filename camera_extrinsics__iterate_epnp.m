%{

Refactored from /mnt/nixbig/ownCloud/project_code/camera_extrinsics__generate_perfect_data.m line 96-123 section "try with a larger model, same dataset"
%}
function [ ...
        models , models_max_diff_SE3_element , ...
        models_extrinsic_estimate_as_local_to_world ... % , models_extrinsic_estimate_as_world_to_local ...  %         , models_best_solution , models_solution_set   
        ] = ...
    camera_extrinsics__iterate_epnp  ( ...
        points_2D_fulldataset_conditioned, points_3D_fulldataset_conditioned, camera_K, ...
        num_RANSAC_iterations, model_size , ...
        TODO_camera_get_pose_transform)
    display(model_size)
    models = zeros(model_size,num_RANSAC_iterations, 'uint32');
    models_max_diff_SE3_element = zeros(1,num_RANSAC_iterations);
    models_extrinsic_estimate_as_local_to_world = zeros(4,4,num_RANSAC_iterations);
    models_best_solution = zeros(1,num_RANSAC_iterations);  
    for ii_=1:num_RANSAC_iterations 
        [ points_3D_model , model_datapoint_indices ] = datasample(points_3D_fulldataset_conditioned, model_size, 2, 'Replace', false);
        points_2D_model = points_2D_fulldataset_conditioned(:,[model_datapoint_indices]);
        if camera_extrinsics__is_sample_degenerate( points_3D_model , points_2D_model ) 
            continue
        end
        models(:,ii_) = model_datapoint_indices;
        while true
            try 
                [R,T,Xc,best_solution, solution_set, Alph]=efficient_pnp_will( points_3D_model' , points_2D_model' , camera_K );  
                break;
            catch
                display('problem')
            end
        end
        %  models_extrinsic_estimate_as_world_to_local = [ R , T  ;  [ 0 0 0 1]  ];
        extrinsic_estimate_as_local_to_world =  vertcat(  horzcat( R', (-(R')) * T )  ,   [ 0 0 0 1]  );    
        models_max_diff_SE3_element(ii_) = max(max(abs(extrinsic_estimate_as_local_to_world - TODO_camera_get_pose_transform)));
        models_extrinsic_estimate_as_local_to_world(:,:,ii_) = extrinsic_estimate_as_local_to_world;
%         models_best_solution(ii_)=best_solution;
%         if 1 == ii_                
%                 display(sprintf(' 1 == ii_  :  solution_set size=%d', size(solution_set,2)));
%                 models_solution_set = repmat( solution_set , [num_RANSAC_iterations,1] );
%         end
%         display(sprintf('ii_=%d, solution_set size=%d', ii_, size(solution_set,2)));
%         models_solution_set(ii_,1) = solution_set;
    end

end