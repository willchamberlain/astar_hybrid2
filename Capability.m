classdef Capability
    properties
        capability_str
        capability_value
    end
    methods
        function obj = Capability(str_, val_)
            obj.capability_str = str_  ; 
            obj.capability_value = val_  ; 
        end
    end
end