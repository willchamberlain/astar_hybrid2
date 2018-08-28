%%  
addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%
%     see  /mnt/nixbig/ownCloud/project_code/temp__VOS_component_image_processing.m

%%
%{
    
%}
clear('var','capability_array')

capability_array = repmat(Capability('camera',ImageSpec) , 3, 1)
capability_array(2) =  Capability('camera2',ImageSpec)
capability_array(3) =  Capability('camera3',ImageSpec)
%   display( capability_array(2).capability_str )


infoRequestByRobotController = InfoRequest('free space')

vosClient = VosClient()

vosClient.fulfillRequest( infoRequestByRobotController , capability_array )



algorithms(1).type='free space'
algorithms(1).subtype='colour model'
algorithms(1).imagespec = ImageSpec()
algorithms(1).imageprocessing = ImageProcessing()
algorithms(1).imagedemand = ImageDemand()
algorithms(1).imagespec.type_name='RGB'
algorithms(1).imagespec.num_channels = 3
algorithms(1).imagedemand.max_scaling_distortion_ratios = [2 1]
algorithms(1).imagedemand.max_scaling_upsample = 2
algorithms(1).imagedemand.max_scaling_downsample = 4

algorithms(2).type='free space'
algorithms(2).subtype='texture model - grey level runlength'
algorithms(2).imagespec = ImageSpec()
algorithms(2).imageprocessing = ImageProcessing()
algorithms(2).imagedemand = ImageDemand()
algorithms(2).imagespec.num_channels = 1
algorithms(2).imagespec.type_name='mono'
algorithms(2).imagedemand.max_scaling_distortion_ratios = [1 1]
algorithms(2).imagedemand.max_scaling_upsample = 1
algorithms(2).imagedemand.max_scaling_downsample = 2


structElement = struct('a1','', 'a2', '', 'a3', '', 'a4', '')
s2 = repmat(structElement, [2,1])
s2(2).a2='bob'
s2(1).a2
s2(2).a2
s2(1).a2=5
s2(1).a2
s2(2).a2

clear('var','algorithms_to_use')

for ii_ = 1: size(algorithms,2) 
    if strcmp(algorithms(ii_).type , 'free space')
        if ~exist('algorithms_to_use','var')
            algorithms_to_use(1) = algorithms(ii_)  
        else
            algorithms_to_use(end+1) = algorithms(ii_)  
        end
    end
end

imageProcessing_rgb_to_mono = ImageProcessing()
imageProcessing_rgb_to_mono.image_demand =  imageProcessing_rgb_to_mono.imageDemand
imageProcessing_rgb_to_mono.image_demand.type_name='RGB|GBR|BGR' 
imageProcessing_rgb_to_mono.image_demand.num_channels=3
imageProcessing_rgb_to_mono.image_demand.


vosClient --> local lookup of algorithms :  for='free space'  : capabilities='camera'
if not found ,  or maybe anyway
    vosClient --> connect to VOS Server and lookup of algorithms :  for='free space'  : capabilities='camera' 
    
    