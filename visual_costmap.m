addpath('/mnt/nixbig/ownCloud/project_code/')

%%

load map1

idisp(map)
map



% layer1:  floorplan: fixed-obstacle: in ROS format: is 255=white=free --> 0=black=occupied
map_im = iread('/mnt/nixbig/ownCloud/project_code/map_S11_cutdown_by_hand.png', 'double');
map_im = 1- map_im;
% [ n , edges ]  =  histcounts(map_im)
% histogram( map_im )
map_layers( : , : , 1 ) = map_im;

camera_layers(:,:,1) = ones(size(map_layers,1), size(map_layers,2));

idisp(map_layers(:,:,1))
%imshow(map_layers(:,:,1))
display_map(map_layers, 1)

map_layers( : , : , 2 ) =  1-iread('/mnt/nixbig/ownCloud/project_code/map_S11_cutdown_by_hand_floorplan.png', 'double');
% [ n , edges ]  =  histcounts( map_layers( : , : , 2 ) )
% histogram( map_layers( : , : , 2 ) )
display_map(map_layers, [1 2])

map_summed = map_layers(:,:,1) + map_layers(:,:,2);
histogram( map_summed )
map = map_summed;
% map = map_summed./size(map_layers,3)
map( map(:,:) > 1 ) = 1;        % cap to max cost 
display_map(map,1)
idisp(map)

%camera frustum --> field of view :  project rectangular pyramid and intersection with the ground plane
%camera resolution --> pixelation : simplify and assume as linear and symmetric around the optical axis :  project cone and intersection with the ground plane

% map is built, now plan on it 

dx = DXform(map);

goal = [10,10];
goal = [50; 140]
% dx.plan(goal)
% animate iterations of cost estimation
dx.plan(goal, 0.025);
dx.plot()

start = [220, 140]
% animate:  dx.path(start);
p = dx.path(start);
dx.plot(p)

% kgauss(5)
% size( kgauss(5) )
% size( kgauss(6) )
% imeshgrid( kgauss(6) )
% scale for the angle of incidence, and rotate and translate to meet the
% optical axis
% scale=[ 2 0  ; 0 1  ]  % if we scale the coordinates in a vector : we need to use interpolation to get the values over a range of coordinates
%  KISS: just use the Gaussian  
% Gaussian is wrong anyway; want an inverse exponential decay 

% observability: approximate as max across the cameras  
camera_layers(:,:,1) = ones(size(map_layers,1), size(map_layers,2));

m = camera_resolution(40);
figure;imshow(m)

next_layer_number = size(camera_layers,3)+1  ;
camera_layers( : , : , next_layer_number )  =  zeros(size(camera_layers,1),size(camera_layers,2))  ;
cam_opt_axis_y = 50; 
cam_opt_axis_x = 70;
camera_layers( ...
        ceil(cam_opt_axis_y-size(m,1)/2):ceil(cam_opt_axis_y-size(m,1)/2)+size(m,1)-1  , ...
        ceil(cam_opt_axis_x-size(m,2)/2):ceil(cam_opt_axis_x-size(m,2)/2)+size(m,2)-1 , ...
        next_layer_number )   =   ...
    camera_layers( ...
        ceil(cam_opt_axis_y-size(m,1)/2):ceil(cam_opt_axis_y-size(m,1)/2)+size(m,1)-1  , ...
        ceil(cam_opt_axis_x-size(m,2)/2):ceil(cam_opt_axis_x-size(m,2)/2)+size(m,2)-1 , ...
        next_layer_number )  + m;  
camera_layers(:,:,next_layer_number)    =  1 - camera_layers(:,:,next_layer_number);

display_map( camera_layers , next_layer_number )   


m = camera_resolution(60);

next_layer_number = size(camera_layers,3)+1  ;
camera_layers( : , : , next_layer_number )  =  zeros(size(camera_layers,1),size(camera_layers,2))  ;
cam_opt_axis_y = 120; 
cam_opt_axis_x = 62;
camera_layers( ...
        ceil(cam_opt_axis_y-size(m,1)/2):ceil(cam_opt_axis_y-size(m,1)/2)+size(m,1)-1  , ...
        ceil(cam_opt_axis_x-size(m,2)/2):ceil(cam_opt_axis_x-size(m,2)/2)+size(m,2)-1 , ...
        next_layer_number )   =   ...
    camera_layers( ...
        ceil(cam_opt_axis_y-size(m,1)/2):ceil(cam_opt_axis_y-size(m,1)/2)+size(m,1)-1  , ...
        ceil(cam_opt_axis_x-size(m,2)/2):ceil(cam_opt_axis_x-size(m,2)/2)+size(m,2)-1 , ...
        next_layer_number )  + m;  
camera_layers(:,:,next_layer_number)    =  1 - camera_layers(:,:,next_layer_number);

display_map( camera_layers , next_layer_number )    

m = camera_resolution(50);
next_layer_number = size(camera_layers,3)+1  ;
camera_layers( : , : , next_layer_number )  =  zeros(size(camera_layers,1),size(camera_layers,2))  ;
cam_opt_axis_y = 120; 
cam_opt_axis_x = 175;
camera_layers( ...
        ceil(cam_opt_axis_y-size(m,1)/2):ceil(cam_opt_axis_y-size(m,1)/2)+size(m,1)-1  , ...
        ceil(cam_opt_axis_x-size(m,2)/2):ceil(cam_opt_axis_x-size(m,2)/2)+size(m,2)-1 , ...
        next_layer_number )   =   ...
    camera_layers( ...
        ceil(cam_opt_axis_y-size(m,1)/2):ceil(cam_opt_axis_y-size(m,1)/2)+size(m,1)-1  , ...
        ceil(cam_opt_axis_x-size(m,2)/2):ceil(cam_opt_axis_x-size(m,2)/2)+size(m,2)-1 , ...
        next_layer_number )  + m;  
camera_layers(:,:,next_layer_number)    =  1 - camera_layers(:,:,next_layer_number);

    
camera_observability_map = ones( size(camera_layers,1) , size(camera_layers,2) );
for ii_ = 1:size(camera_layers,1) 
    for jj_ = 1:size(camera_layers,2)
        camera_observability_map(ii_,jj_) = min(  camera_layers( ii_ , jj_ , : )  )   ;
    end
end
% imshow(camera_observability_map)
observability_weighting = 0.5;
camera_observability_map = camera_observability_map.* observability_weighting;
imshow(camera_observability_map)
% histogram(camera_observability_map)

% combine the obstacle map and the observability map into a cost map for D*
% etc.
% cost_map = zeros( size(map_layers,1) , size(map_layers,2), 'double' );
% 
% for ii_ = 1:size(camera_layers,1) 
%     for jj_ = 1:size(camera_layers,2)
%         cost_map(ii_,jj_) = max(  camera_observability_map(ii_,jj_) , map(ii_,jj_)  )   ;
%     end
% end
% imshow(cost_map)
% idisp(cost_map)
% 
% cost_map = 1- cost_map;



% dx = DXform(cost_map);
dx = DXform(map);

goal = [50; 140]
% dx.plan(goal)
% animate iterations of cost estimation
%dx.plan(goal, 0.0125);
dx.plan(goal);
dx.plot();
start = [220, 140]
dx.path(start);
% p = dx.path(start);
% dx.plot(p)
% 
% dx.occgrid
% dx.distancemap
% figure;histogram(dx.distancemap)  
%    max(max(dx.distancemap))
% figure;histogram(dx.distancemap.*0.5)
% figure;histogram(127+dx.distancemap.*0.5);  255*0.5
%       max(max(127+dx.distancemap.*0.5))
% figure;histogram(camera_observability_map*100)
% size(dx.distancemap)
% size(camera_observability_map)
figure; idisp(dx.distancemap)
new_distancemap = 127+(dx.distancemap.*0.5) ;
% figure;histogram(new_distancemap)
figure; imshow(new_distancemap, [0,255])
new_distancemap = new_distancemap - ( (1-camera_observability_map)*127);
figure; imshow(new_distancemap, [0,255])
figure;histogram(new_distancemap)
% for ii_ = 1:size(camera_layers,1) 
%     for jj_ = 1:size(camera_layers,2)
%         if dx.occgrid(ii_,jj_) == 1 %&& new_distancemap(ii_,jj_) < 255
%             for i_ = -1:1
%                 for j_ = -1:1
%                     new_distancemap(ii_+i_,jj_+j_) = 255  ;
%                 end
%             end
%         end
%     end
% end
idisp(new_distancemap)
dx.distancemap = new_distancemap;
idisp(dx.distancemap)

start = [220, 140]
% animate:  dx.path(start);
p = dx.path(start);
dx.plot(p)

%{
% need to compile some mex files
% need to use GCC4.7 
% need to find_ gcc 4.7   -->  https://www.cyberciti.biz/faq/linux-how-to-check-what-compiler-is-running-installed/
% bash: dpkg --list   | grep -i compiler |   grep -i gcc
% need directory location --> 
% which g++
% /usr/bin/g++
%  $ ls -la /usr/bin/g++
% lrwxrwxrwx 1 root root 21 Nov 21  2016 /usr/bin/g++ -> /etc/alternatives/g++
%  $ ls -la  /etc/alternatives/g++
% lrwxrwxrwx 1 root root 14 Dec 23 15:30 /etc/alternatives/g++ -> /usr/bin/g++-5
%  $ ls -la /usr/bin/g++-5
% -rwxr-xr-x 1 root root 890704 Sep  5  2016 /usr/bin/g++-5
%  $ ls -la /usr/bin/g++*
% lrwxrwxrwx 1 root root     21 Nov 21  2016 /usr/bin/g++ -> /etc/alternatives/g++
% -rwxr-xr-x 1 root root 259176 Mar 20  2014 /usr/bin/g++-4.4
% -rwxr-xr-x 1 root root 578808 Mar 21  2014 /usr/bin/g++-4.7
% -rwxr-xr-x 1 root root 775888 Dec  1  2015 /usr/bin/g++-4.8
% -rwxr-xr-x 1 root root 841648 Sep  5  2016 /usr/bin/g++-4.9
% -rwxr-xr-x 1 root root 890704 Sep  5  2016 /usr/bin/g++-5
% 
% try update-alternatives
%           https://askubuntu.com/questions/26498/choose-gcc-and-g-version
%           https://codeyarns.com/2015/02/26/how-to-switch-gcc-version-using-update-alternatives/
%
%  still problems: C files with C++ comments: need to use -std=gnu99
%   https://au.mathworks.com/matlabcentral/answers/137228-setup-mex-compiler-for-r2014a-for-linux
%   https://xcorr.net/2010/05/07/getting-gcc-to-work-with-matlab-r2009b-on-linux-ubuntu-karmic-to-compile-mex-files/
%
%       cat     ~/.matlab/R2016a/mex_C_glnxa64.xml
%       cat     ~/.matlab/R2016a/mex_C++_glnxa64.xml
%
    setenv('CFLAGS',' -std=gnu99 ')
    getenv('CFLAGS')
    NOPE: running 

backup file
        $ cp ~/.matlab/R2016a/mex_C++_glnxa64.xml ~/.matlab/R2016a/mex_C++_glnxa64.xml_bak_2018_02_22_1106
edit file       -->     https://xcorr.net/2010/05/07/getting-gcc-to-work-with-matlab-r2009b-on-linux-ubuntu-karmic-to-compile-mex-files/
         ~/.matlab/R2016a/mex_C++_glnxa64.xml
        -ansi       -->     -std=c99

        and in Matlab
            addpath( '/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/vision/mex/' )
and _that_ finally worked

/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/startup_rvc.m
%}


