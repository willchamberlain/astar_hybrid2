classdef (Abstract) ImageSpecA
    properties
        type_name 
        width
        height
        num_channels      
        FoV_width_angle
        FoV_height_angle

        dim
        resolution_ratio_vs_height
        resolution_ratio_vs_height9

    end
    %    methods
    %       function r = roundOff(obj)
    %          r = round([obj.Value],2);
    %       end
    %       function r = multiplyBy(obj,n)
    %          r = [obj.Value] * n;
    %       end
%    end
end