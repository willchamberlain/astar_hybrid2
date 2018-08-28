classdef ImageProcessing < ImageProcessingA
    methods
        function imageDemand_partial__ = imageDemand(obj) 
            imageDemand_partial__ = ImageDemand();  
            imageDemand_partial__.max_scaling_distortion_ratios = obj.max_scaling_distortion_ratios_default  ;
            imageDemand_partial__.max_scaling_upsample = obj.max_scaling_upsample_default  ;
            imageDemand_partial__.max_scaling_downsample = obj.max_scaling_downsample_default  ;
        end
    end
end