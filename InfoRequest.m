classdef InfoRequest
    properties
        requestType
    end
    
    methods
        
        function obj = InfoRequest(request_desc_str_)
            obj.requestType = request_desc_str_  ;
        end
        
    end
end