classdef Wall
    properties
        shape
        location
    end
    methods
        function obj = Wall(shape, location)
            obj.shape = shape  ;
            obj.location = location  ;
        end
    end
end 