classdef (Abstract) ImageProcessingA
    properties
        max_scaling_distortion_ratios_default = [ 1 1 ] ;
        max_scaling_upsample_default = 1  ;
        max_scaling_downsample_default = 8  ;
        image_demand  ; 
    end
    methods
        imageDemand_partial__ = imageDemand(obj) 
    end
end