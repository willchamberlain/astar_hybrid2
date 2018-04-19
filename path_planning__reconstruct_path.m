
% return 2xn
function [ total_path_goal_to_start__  , total_path_start_to_goal__ ] = path_planning__reconstruct_path(came_from_, current_node_)
    total_path_goal_to_start__(:,1) = current_node_    ;       
    while true        
        current_node_ = came_from_( :,current_node_(1),current_node_(2) )    ;
        if isempty(current_node_) | [0;0] == current_node_    ;
            return
        end
        total_path_goal_to_start__(:,end+1) = current_node_    ;                
        total_path_start_to_goal__ = flip(total_path_goal_to_start__,2);
    end
end