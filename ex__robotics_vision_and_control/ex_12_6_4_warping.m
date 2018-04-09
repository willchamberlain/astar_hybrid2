lena = iread('lena.pgm');

%% p338

[Ui, Vi] = imeshgrid(lena);                 % indices 

[Uo, Vo] = imeshgrid(400,400);         % indices

% u'  = f_u(u,v) , v' = f_v(u,v)
% e.g. 
% u' = u/4 + 100 , v' = v/4 + 200

% find the input image pixel: inverse function
% u = (u'-100) * 4
% v = (v'-200) *4
U = (Uo - 100)*4;                               % indices : U is the index in Ui that maps to Uo
V = (Vo - 200)*4;                               

interp2  % 2D interpolation
lena_small = interp2(Ui, Vi, idouble(lena), U,V);
idisp(lena_small)

%% p340

% Bouguet's image -  distorted = iread('Image18.tif', 'double');
distorted = iread('/mnt/nixbig/ownCloud/project_AA1__2_extrinsics_calibration/project_AA1_2_extrinsics__phone_data_recording__copied_bc/VOS_data_2018_01_21_16_51_11_calibration_pattern_Galaxy3_home/2018_01_21_16_55_01_BoofCV_image.png','double');
distorted = distorted(:,:,1);   % cheap mono
[Ui,Vi] = imeshgrid(distorted);
Uo = Ui; Vo = Vi;
% load Bouguet
% my calibration of GalaxyIII
k = [0.004180016841640, 0.136452931271259, -0.638647134308425]; % radial
p = [-0.001666231998527, -0.00008160213039217031]; % tangental
Principal_point=    [1.768267919600727e+02, 1.467681514313797e+02];
u0 = Principal_point(1)
v0 = Principal_point(2)
Focal_length =  [3.229596901156589e+02, 3.238523693059909e+02]
fpix_u = Focal_length(1)
fpix_v = Focal_length(2)

% Convert pixel coordinates to normalized image coordinates.
% In units of metres with respect to the camera's principal point.
 u = (Uo-u0) / fpix_u;
 v = (Vo-v0) / fpix_v;
% radial distance of the pixels from the principal point
 r = sqrt( u.^2 + v.^2 );
% the pixel coordinate errors due to distortion are
delta_u = u .* (k(1)*r.^2 + k(2)*r.^4 + k(3)*r.^6) + ...
  2*p(1)*u.*v + p(2)*(r.^2 + 2*u.^2);
delta_v = v .* (k(1)*r.^2 + k(2)*r.^4 + k(3)*r.^6) + ...
  p(1)*(r.^2 + 2*v.^2) + 2*p(2)*u.*v;
% undistorted pixel coordinates in metric units are
ud = u + delta_u;       vd = v + delta_v;
% convert back to pixel coordinates
 U = ud * fpix_u + u0;
 V = vd * fpix_v + v0;
% apply the warp
undistorted = interp2(Ui, Vi, distorted, U, V);
idisp(undistorted)          % not perfect, but maybe adequate 


 
 
 
 