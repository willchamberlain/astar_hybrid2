classdef PathPlanning_Astar
    properties
        start
        goal
        map
    end
    methods
        function map__ = map_to_plan(obj, map_)
            obj.map = map_;
            map__ = obj.map;
        end        
        function est__ = heuristic_cost_estimate(obj, start_, goal_)
            est__ = norm(goal_-start_,2);
        end
        function path__ = reconstruct_path(came_from_, current_node_index_)
            path__ = 'reconstruct_path';
        end
        function path__ = plan_path(obj, start_, goal_)
            obj.start = start_;
            obj.goal = goal_;
            
            closed_set = zeros(size(obj.map), 'uint32')  ;  % as a map: 1 is closed, zero is not-closed
            
            open_set = zeros(size(obj.map), 'uint32')  ;  % as a map: 1 is in-the-open-set , zero is not-in-the-open_set
            open_set(obj.start(1),obj.start(2)) = 1 ;
            
            % For each node, which node it can most efficiently be reached from
            came_from = zeros(size(obj.map), 'uint32')  ;
            
            % For each node, the cost of getting from the start node to that node.
            g_score = inf( size(obj.map) )  ;
            g_score(obj.start(1),obj.start(2)) = 0  ;
            
            % For each node, the total cost of getting from the start node to the goal
            %   by passing by that node. That value is partly known, partly heuristic.
            f_score = inf( size(obj.map) )  ;
            f_score(obj.start(1),obj.start(2)) = obj.heuristic_cost_estimate(obj.start, obj.goal)  ;            
            
            while sum(sum(open_set>0))
                 [row_,col_]=find(f_score == min(min(f_score(open_set>0))))  ;
                if size(row_,1)>1
                    row_=row_(1);
                    col_ = col_(1);
                end
                current_node_index = [col_,row_]  ;
                current_node = open_set( current_node_index )  ;
                if current_node_index == obj.goal
                    path__ = reconstruct_path(came_from, current_node_index)
                    return
                end
                open_set(open_set(:,1)==current_node_index(1) & open_set(:,2)==current_node_index(2),:) = [];
                
                if isempty(closed_set) ; closed_set=current_node_index;
                else closed_set(end+1,:) = current_node_index;
                end 
            end
            
            path__ = 'NOT IMPLEMENTED YET';
        end
    end
end