classdef Task  % is also a TaskExecution for now
    properties
        target_model
        
        target_pose
        target_pose_most_likely
        target_pose_uncertainty
    end
    methods
        function obj = Task(target_model)
            obj.target_pose_uncertainty = 1  ;
            if nargin>0
                obj.target_model = target_model  ;
            end
        end
        function is_target_pose_known__ = is_target_pose_known(obj)
            is_target_pose_known__ = obj.target_pose_uncertainty > 0.2  ;
        end
        
    end
end