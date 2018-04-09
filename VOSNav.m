%{
Planning and navigation implementation:  
    - plan(goal) creates a plan
    - n = next(ds, current) is called repeatedly by Navigation.path(nav,
    start) to generate a path: next navigation waypoint given current
    location
%}
classdef VOSNav
    
    properties
    end
    
    methods
        
        function plan(vosNav, goal)
        end
        
        function n = next
        end
        
        
    end
end