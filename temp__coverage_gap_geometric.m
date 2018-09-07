

%{
Thursday 2018_09_06

Coverage gaps represented as geometric objects - polyhedra in 2D - and plan across them via shortest-paths.

Hypothesis:  

Contribution / significance: baseline for comparison of efficiency for more complex gap representations

Contribution / significance: 


Problem case : fields of view separated by a corner in a corridor - needs an intermmediate point..

%}

%%

addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%

% addpath( '/mnt/nixbig/downloads/matlab_DistBetween2Segment/DistBetween2Segment/' )
addpath(  '/mnt/nixbig/downloads/matlab_DistBetween2Segment/DistBetween2Segment/'  )

%%

%{
Uncertainty propogation through robot's pose estimation during motion 
From EKF is   command_action



EKF comments: 
%    veh = Vehicle(V);
%    veh.add_driver( RandomPath(20, 2) );
%    ekf = EKF(veh, V_est, P0);
% We run the simulation for 1000 time steps
%    ekf.run(1000);
% then plot true vehicle path
%    veh.plot_xy('b');
% and overlay the estimated path
%    ekf.plot_xy('r');
% and overlay uncertainty ellipses at every 20 time steps
%    ekf.plot_ellipse(20, 'g');
% We can plot the covariance against time as
%    clf
%    ekf.plot_P();

%}