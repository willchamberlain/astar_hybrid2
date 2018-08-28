%%
%{
VOS Client and Smart Camera 
- same interface 
- Smart Cameras _are_ Clients with no operational vision tasks



 VOS Component Image Processing 
-  demand-driven image processing
    -- dataflow
    -- memory 
        --- intra-process : retain for later steps
        --- inter-process : make available for
        --- cache 
-  DSL / Syn code generation / generation-gap code generation / macro-expansion 
-  parse pseudo-code into statements 
-  

Then refactor into a DSL for e.g. C++ and Python code generation. 
%}


%%
im_ = imread('/home/will/Desktop/field_of_sheep_1024x768__lagill6_wordpress__Louis_Gill_2013.jpg')  ;
camera_image_type = 'RGB'  ;
camera_width = 1024  ;
camera_height = 768  ;
camera_dim = [ camera_width camera_height ]  ;
camera_res_ratio_vs9 = [ 9*(camera_width/camera_height) , 9 ]  ;
%  --> set of camera available resolutions, per Android
%  ANd/OR --> set of camera spatial/angular resolutions + field of view angles per zoom - if zoom in, get better spatial resolution, but lower FoV angles

source_code_string = 'image( colour=gray_unint32 , dim=640x480)' ;
source_code_string = 'image( colour=gray , dim=640x480)' ;

[matches, tokens] =  regexp(source_code_string,'colour=([\w_]+)', 'match',  'tokens')
image_type = tokens{1}{1}  ;

[matches, tokens] =  regexp(source_code_string,'dim=(\d+)x(\d+)', 'match',  'tokens')
image_width = str2double(  tokens{1}{1}  )  ;
image_height = str2double(  tokens{1}{2}  )  ;
image_dim = [ image_width image_height ]  ;
image_res_ratio_vs9 = [ 9*(image_width/image_height) , 9 ]  ;

scaling_ratios = image_dim ./ camera_dim  ;

    max_scaling_distortion_ratios_default = [ 1 1 ] ;
    max_scaling_distortion_ratios = max_scaling_distortion_ratios_default  ;  
    scaling_distortion_ratios = camera_res_ratio_vs9 ./ image_res_ratio_vs9  ;
    scaling_distortion_ratios_acceptable = scaling_distortion_ratios <= max_scaling_distortion_ratios  ;
    scaling_distortion_ratios_acceptable = scaling_distortion_ratios_acceptable(1) && scaling_distortion_ratios_acceptable(2);

    max_scaling_upsample_default = 1  ;
    max_scaling_upsample = max_scaling_upsample_default  ;
    scaling_upsample_acceptable = image_dim./camera_dim <= [max_scaling_upsample_default max_scaling_upsample_default]
    scaling_upsample_acceptable = scaling_upsample_acceptable(1) && scaling_upsample_acceptable(2)

    max_scaling_downsample_default = 8  ;
    max_scaling_downsample = max_scaling_downsample_default ;
    scaling_downsample_acceptable = camera_dim./image_dim <= [ max_scaling_downsample max_scaling_downsample ]
    scaling_downsample_acceptable = scaling_downsample_acceptable(1) && scaling_downsample_acceptable(2)
    
    
% QUESTION: if upscaling, upscale in RGB first, then convert to monochrome ?
%       equiv: is upscaling and im_output_format.num_channels < im_input_format.num_channels , upscale in each channel of im_input first, then convert to im_output channels ?
if strcmpi(image_type, camera_image_type)
    colour_mapped_image = im_;
    display('recoloured: colur is the same')
else 
    if strcmpi('RGB',camera_image_type)
        if strcmpi('gray',image_type) || strcmpi('grey',image_type) 
            colour_mapped_image = rgb2gray(im_)  ;
            % NOTE : ASSUMPTION : RGB camera is human-parameterised so that the standard RGB-to-monochrome converstion is legitimate (  0.2989 * R + 0.5870 * G + 0.1140 * B  )
            display('recoloured RGB 2 grey')
        end
    end
end
%       IMRESIZE(A, SCALE, METHOD) 
%       IMRESIZE(A, [NUMROWS NUMCOLS], METHOD),
if scaling_ratios == [1 1]
    scaled_image = colour_mapped_image ; 
    display('rescaled: scale is the same')
else
    scaled_image = imresize( colour_mapped_image , [ image_height , image_width ]  )  ;
    % NOTE : ASSUMPTION : use icubic interpolation and antialiasing  (the imresize defaults
    display(sprintf('rescaled: rescaled from %ix%i to %ix%i',  camera_width,camera_height  ,   image_width,image_height ))
end


%{
    See lab book for 2018_08_21
    --> monochrome  --> mono source | RGB source -> mono | hyperspectral source -> visible mono
        ==> need to understand sources at least a little - by spec or by evidence (output history, or ask for output then inspect)
    --> 4:3  &  large_dim == 640  (=> small_dim == 480)  
        _Displays_   -   _not_ cameras  -  as w:h
            --- claimed: 5:4 , 4:3 , 3:2 , 8:5 , 16:10 , 5:3 , 16:9 , 17:9 , 21:9  -  see https://en.wikipedia.org/wiki/Display_resolution#/media/File:Vector_Video_Standards8.svg
            --- normalised to x:9
                --- 11.25:9 , 12.0:9 , 13.5:9 , 14.4:9  , 14.4:9  , 16:9 , 17.0:9 , 21.0:9
            --- actual: 12.00:9 , 16.00:9 , 13.50:9 , 14.40:9 , 15.00:9  , 16.00:9 , 17.06:9 , 21.33:9|21.50:9 ,
    --> max_downscale == default == 4:1
    --> max_upscale == default == 1:1 
    --> max_scaling_distortion == default == [ 1:1  1:1 ]    
        ????  infer from downstream algorithms - many convnet are happy with e.g. 1920:1080 -> 120:120 e.g. 16:9 --> 9:9 = [1.78 1] distortion ( in [ w h ]  )
    --> max_trim == default = 'centred':0:0:0:0
    --> not even mentioning pixel noise == default == ???
    --> not even mentioning distortion == default == corrected ???
%}

regexp