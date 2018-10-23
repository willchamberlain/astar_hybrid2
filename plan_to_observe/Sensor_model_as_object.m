%%  Sensor model : 
%       uncertainty vs. distance of feature to camera optical centre
%       uncertainty vs. angle of robot feature to camera optical axis
%%
%           addpath('/mnt/nixbig/ownCloud/project_code/plan_to_observe/')     
%%  

classdef Sensor_model_as_object  <  handle    
   properties (Constant)
      NOT_AT_GOAL = 1000;
      AT_GOAL = 2000;
   end   
    properties
        floorplan
        payoffs_map
        start
        goal
        robot_size
        robot_location
        
        need_to_calc_costmap = true  ;
        
        as;
    end
    methods
        function obj = Sensor_model_as_object(floorplan_, payoffs_map_, start_, goal_, robot_size_)
            obj.floorplan=floorplan_ ;
            obj.payoffs_map = payoffs_map_ ;
            obj.start = start_ ;
            obj.robot_location = obj.start ;
            obj.goal = goal_;
            obj.robot_size = robot_size_ ;
        end
        
        function path__ = planningStep(obj, new_start_location_)
            if nargin > 1
                display('New start for planning:')  ;
                display(new_start_location_)  ;
                obj.start = new_start_location_  ;
            end            
            if obj.need_to_calc_costmap
                obj.need_to_calc_costmap = false  ;                
                % map_1 is the base map passed to Navigation; remains zeros to avoid problems between Navigation and DStarMoo:  belongs to DStarMoo
                map_1 = zeros(size(obj.payoffs_map,1),size(obj.payoffs_map,2))  ;  % no walls
                inflated_flooplan = ismooth(obj.floorplan,obj.robot_size)  ;
                obj.as = DstarMOO(map_1,inflated_flooplan)  ;                   
                for ii_ = 1:size(obj.payoffs_map,3)
                    costs_map = max(max(obj.payoffs_map(:,:,ii_))) - obj.payoffs_map(:,:,ii_) ;
                    costs_map = squeeze(costs_map ) ;
                    normA = costs_map - min(min((costs_map)));
                    normA = normA ./ max(max(normA))  ;
                    obj.as.addCost(ii_,normA)  ;        % add additional cost layer
                end                
                obj.as.plan(obj.goal, 2+size(obj.payoffs_map,3) , obj.start )  ;
            end
            path__ = obj.as.path(obj.start)  ;    % plan solution path star-goal, return path             
        end
        
        function goal_moved(obj, new_goal_)
            if ~isequal(obj.goal, new_goal_)
                obj.need_to_calc_costmap = true;
                obj.goal=new_goal_;
            end
        end
        
        function status__ = moveAStep(obj)
            path_ = obj.planningStep(obj.robot_location)  ;
            if size(path_,1)>0
                obj.robot_location = path_(1,:)  ;
                status__ = Sensor_model_as_object.NOT_AT_GOAL;
            else
                if obj.robot_location == obj.goal
                    status__ = Sensor_model_as_object.AT_GOAL;                
                else
                    display('Problem: path is zero length but robot is not located at the goal.')
                end
            end
        end
    end
end










