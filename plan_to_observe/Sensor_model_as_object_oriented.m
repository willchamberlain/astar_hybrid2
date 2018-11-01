%%  Sensor model : 
%       uncertainty vs. distance of feature to camera optical centre
%       uncertainty vs. angle of robot feature to camera optical axis
%%
%           addpath('/mnt/nixbig/ownCloud/project_code/plan_to_observe/')     
%%  

classdef Sensor_model_as_object_oriented  <  handle    
   properties (Constant)
      NOT_AT_GOAL = 1000;
      AT_GOAL = 2000;
   end   
    properties
        floorplan
        payoffs_map
        normals_map
        start
        goal
        robot_size
        robot_location
        robot_id
        
        need_to_calc_costmap = true  ;
        
        as;
        motion_cost = 1;
        vel = 1;
        current_path;
    end
    methods
        function obj = Sensor_model_as_object_oriented(robot_id_, floorplan_, payoffs_map_, normals_map_, start_, goal_, robot_size_, varargin)
            obj.robot_id = robot_id_ ;
            obj.floorplan=floorplan_ ;
            obj.payoffs_map = payoffs_map_ ;
            obj.normals_map = normals_map_;
            obj.start = start_ ;
            obj.robot_location = obj.start ;
            obj.goal = goal_;
            obj.robot_size = robot_size_ ;
            if ~isempty(varargin) && size(varargin,2) > 0 
                obj.motion_cost = varargin{1} ;
                display(sprintf('motion_cost = %f',obj.motion_cost))
            end
            if ~isempty(varargin) && size(varargin,2) > 1 
                obj.vel = varargin{2} ;
                display(sprintf('vel = %f',obj.vel))
            end
        end
        
        function path__ = planningStep(obj, new_start_location_)
            if nargin > 1
                display(sprintf('%s : _old_ start for planning:',obj.robot_id))  ;
                display(obj.start)  ;
                display(sprintf('%s : new start for planning:',obj.robot_id))  ;
                display(new_start_location_)  ;
                obj.start = new_start_location_  ;                
                display(sprintf('%s : current goal for planning:',obj.robot_id))  ;
                display(obj.goal)  ;
            end
            if isequal(size(obj.start),size(obj.goal))
                if obj.start == obj.goal
                    display(sprintf('At goal at %i,%i : should have no planning to do',obj.goal(1),obj.goal(2)));
                    obj.current_path = []  ;    % plan solution path star-goal, return path             
                    path__ = obj.current_path;
                    return
                end
            else
                if obj.start == obj.goal'
                    display(sprintf('At goal at %i,%i : should have no planning to do',obj.goal(1),obj.goal(2)));
                    obj.current_path = []  ;    % plan solution path star-goal, return path             
                    path__ = obj.current_path;
                    return
                end
            end
            if obj.need_to_calc_costmap
                obj.need_to_calc_costmap = false  ;                
                % map_1 is the base map passed to Navigation; remains zeros to avoid problems between Navigation and DStarMoo:  belongs to DStarMoo
                map_1 = obj.floorplan;   %  zeros(size(obj.payoffs_map,1),size(obj.payoffs_map,2))  ;  % no walls
                %  inflated_flooplan = ismooth(obj.floorplan,obj.robot_size)  ;
                inflated_flooplan = obj.floorplan;
                obj.as = DstarMOO_oriented(map_1,inflated_flooplan, 'motion_cost', obj.motion_cost)  ;                   
                for ii_ = 1:size(obj.payoffs_map,3)
                    costs_map = max(max(obj.payoffs_map(:,:,ii_))) - obj.payoffs_map(:,:,ii_) ;
                    costs_map = squeeze(costs_map ) ;
                    normA = costs_map - min(min((costs_map)));
                    normA = normA ./ max(max(normA))  ;
                    obj.as.addCost(ii_,normA)  ;        % add additional cost layer
                    obj.as.addNormal(ii_,obj.normals_map(:,:,ii_))
                end                
                num_layers = 2+size(obj.payoffs_map,3) ;
                display(sprintf('planningStep: num_layers=%i',num_layers))  ;
                obj.as.plan(obj.goal, 2+size(obj.payoffs_map,3) , obj.start )  ;
            end
            
            obj.current_path = obj.as.path(obj.start)  ;    % plan solution path star-goal, return path             
            path__ = obj.current_path;
        end
        
        function changeGoal(obj, new_goal_)
            if ~isequal(obj.goal, new_goal_)
                obj.need_to_calc_costmap = true;
                obj.goal=new_goal_;
            end
        end
        
        function status__ = moveAStep(obj)
            path_ = obj.planningStep(obj.robot_location)  ;
            if size(path_,1)>=obj.vel
                obj.robot_location = path_(obj.vel,:)  ;
                status__ = Sensor_model_as_object_oriented.NOT_AT_GOAL;
                temp_goal_to_compare = obj.goal;
                if ~isequal( size(obj.robot_location) , size(obj.goal) )
                    temp_goal_to_compare = obj.goal';
                end
                %display: display('norm_2(temp_goal_to_compare - obj.robot_location , 2) ::  size(path_,1)>0')  ;
                %display: display(  norm_2(temp_goal_to_compare - obj.robot_location , 2) )  ;
                %display: isequal(obj.robot_location , temp_goal_to_compare)
                %display: norm_2(temp_goal_to_compare - obj.robot_location , 2) < 2
                if  isequal(obj.robot_location , temp_goal_to_compare) ...  
                    ||   norm_2(temp_goal_to_compare - obj.robot_location , 2) < 2 %   edge case: robots alternating position: change at-goal to proximity rather than on top of  i.e. Euclidean distance < 2.
                    display(sprintf('Robot %s at goal', obj.robot_id)) ;
                    status__ = Sensor_model_as_object_oriented.AT_GOAL;                
                end
            else
                %display: display('obj.robot_location')
                %display: obj.robot_location
                %display: display('obj.goal')
                %display: obj.goal
                temp_goal_to_compare = obj.goal;
                if ~isequal( size(obj.robot_location) , size(obj.goal) )
                    temp_goal_to_compare = obj.goal';
                end
                        %[y,i]=max(size( [45 12] - [-12 45] ))
                %display: display('norm_2(temp_goal_to_compare - obj.robot_location , 2)')  ;
                %display: display(  norm_2(temp_goal_to_compare - obj.robot_location , 1) )  ;
                if  isequal(obj.robot_location , temp_goal_to_compare) ...  
                    ||   norm_2(temp_goal_to_compare - obj.robot_location , 2) < 2 %   edge case: robots alternating position: change at-goal to proximity rather than on top of  i.e. Euclidean distance < 2.
                    display(sprintf('Robot %s at goal', obj.robot_id)) ;
                    status__ = Sensor_model_as_object_oriented.AT_GOAL;                
                else
                    display('Problem: path is zero length but robot is not located at the goal.')
                end
            end
        end
    end
end










