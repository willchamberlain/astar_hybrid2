% EPnP as-is puts control points at the centroid of the 3D points and the unit
% vectors of the PCA of the 3D points.

% For RANSAC, and given the constraints on the experiment - rigid robot, planar robot, 
% planar movement with rotation, fixed cameras,  how do I determine which sectors of the 
% data to sample from?  - more efficient to pick samples which support the
% algorithm than to try to outnumber the outliers.  ( ?? ) 
%
% Really want to cluster with e.g. mean shift - deals with the distribution
% across the covariance of dimensions.   Maybe run for a certain number of
% iterations and then find neighbours.
% Really want a very fast, simple method e.g. equal-sized bins over a range - easy
% to understand, deals with this data OK - I THINK.
% NO - Outer-index per dimension and 

figure; plot(points_3D_reported_FLU(3,:),'rx')
points_3D_reported_FLU(:,  points_3D_reported_FLU(3,:)>0.7  )

% d1_n  has 3 elements, and the middle one is empty --> 2 populations 
% d2_n  has 4 elements and the none are empty  ~~> continuous population
% --> sample at random ?? sample from 2/4/8 bins ?? sample with minimum
% distance ??
% d2_n  has 4 elements and the none are empty  ~~> continuous population
% --> sample at random ?? sample from 2/4/8 bins ?? sample with minimum
% distance ??
[d1_n, d1_edges] = histcounts(points_3D_reported_FLU(3,:)) 
[d2_n, d2_edges] = histcounts(points_3D_reported_FLU(2,:))
[d3_n, d3_edges] = histcounts(points_3D_reported_FLU(1,:))

randi(points_3D_reported_FLU,1,1)
randi( size(points_3D_reported_FLU,2)  ,1,1)
points_3D_reported_FLU(:,randi( size(points_3D_reported_FLU,2)  ,1,1))


0.9*1.642219750065936 <   points_3D_reported_FLU(1,:)  & points_3D_reported_FLU(1,:) < 1.1*1.642219750065936


points_3D_reported_FLU(1,:) > 1.1 * 1.642219750065936
points_3D_reported_FLU(1,:) < 0.9 * 1.642219750065936




0.9* 0.355000000000000 <   points_3D_reported_FLU(3,:)  & points_3D_reported_FLU(3,:) < 1.1* 0.355000000000000

% empty
points_3D_reported_FLU__subset_d3_less = points_3D_reported_FLU(:,  0.9* 0.355000000000000 >   points_3D_reported_FLU(3,:) )  

points_3D_reported_FLU__subset_d3_more = points_3D_reported_FLU(:,  1.1* 0.355000000000000 <   points_3D_reported_FLU(3,:) )  

%-- next cycle 

points_3D_reported_FLU__working_set = points_3D_reported_FLU__subset_d3_more

%------  next cycle 

points_3D_reported_FLU__working_set(randi( size(points_3D_reported_FLU__working_set,2)  ,1,1))

more_ratio = 1.1
less_ratio = 0.9
more_val = more_ratio * 0.645000000000000
less_val = less_ratio * 0.645000000000000

% all --> done 
less_val <   points_3D_reported_FLU__working_set(3,:)  & points_3D_reported_FLU__working_set(3,:) < more_val

