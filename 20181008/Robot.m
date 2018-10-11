classdef Robot
   properties
      robot_visual_model
      task
      pose
      pose_most_likely
      pose_uncertainty
   end
   methods
       function obj = Robot(robot_model, task)
           obj.pose_uncertainty = 1;
           if nargin>0
                obj.robot_visual_model = robot_model ;
           end
           if nargin>1
                obj.task = task ;
           end
       end
       function is_localised__ = is_localised(obj)
           is_localised__ = obj.pose_uncertainty > 0.2 ;
       end
   end
end