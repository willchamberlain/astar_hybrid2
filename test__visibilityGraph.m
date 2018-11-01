%%
%  http://www.cs.bilkent.edu.tr/~culha/cs548/hw3/
addpath( '/mnt/nixbig/ownCloud/project_code/3rd_party/visibilitygraph_culha/' )
%%
visibilityGraph  % http://www.cs.bilkent.edu.tr/~culha/cs548/hw3/


%%
%   https://au.mathworks.com/matlabcentral/fileexchange/62227-raycasting
addpath( '/mnt/nixbig/ownCloud/project_code/3rd_party/RayCasting/' )
%%
RayCasting
%%
RayCasting_WillC
%%  Want : where could_ the robot go to see a volume that the target might be in, with a limited range sensor?
%   Could approximate sensor as circle, convolve with the uncertainty bound, and then remove the parts 
%   Could sample from the uncertainty bound of the target and apply the sensor model to each of those, culling for obstructions.
%       Could turn it over: for each cell within range-limited-by-sensor-model check LoS;  also mark the cells between as blocked for efficiency _on 2nd version_.
%   Could do it parametrically: define the constraints and solve.


im_ = gauss2d(ones(41),[12,9], [21,21]);  size(im_)
idisp(im_)
im_2 = im_;
im_2(im_<0.7*max(im_(:))) = 0  ;  size(im_2)
idisp(im_2 )


sensor_im_ = gauss2d(ones(41),[35,35], [21,21]);  
idisp(sensor_im_)
sensor_im_2 = sensor_im_;
sensor_im_2(sensor_im_<0.9*max(sensor_im_(:)))  = 0 ; 
idisp(sensor_im_2)
figure; surf(sensor_im_2)

i3 = conv2(sensor_im_2,im_2,'full')  ;
size(i3)
surf(i3)



im_ = gauss2d(ones(41),[1,1], [21,21]);  size(im_)
idisp(im_)
im_2 = im_;
im_2(im_<0.7*max(im_(:))) = 0  ;  size(im_2)
idisp(im_2 )


sensor_im_ = gauss2d(ones(41),[35,35], [21,21]);  
idisp(sensor_im_)
sensor_im_2 = sensor_im_;
sensor_im_2(sensor_im_<0.9*max(sensor_im_(:)))  = 0 ; 
idisp(sensor_im_2)
figure; surf(sensor_im_2)

i3 = conv2(sensor_im_2,im_2,'full')  ;
size(i3)
surf(i3)

%  given that many cells to check against it's going to explode  

bes