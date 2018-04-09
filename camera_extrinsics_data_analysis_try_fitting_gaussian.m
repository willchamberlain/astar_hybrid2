figure
subplot(2,3,1)
histogram(world_to_camera_trans_hist(:,1),20)
subplot(2,3,2)
histogram(world_to_camera_trans_hist(:,2),20)
subplot(2,3,3)
histogram(world_to_camera_trans_hist(:,3),20)
% subplot(2,3,4)
% histogram(world_to_camera_rotm_hist(:,1),20)
% title('rotation - X')
% subplot(2,3,5)
% histogram(world_to_camera_rotm_hist(:,2),20)
% title('rotation - Y')
% subplot(2,3,6)
% histogram(world_to_camera_rotm_hist(:,3),20)
% title('rotation - Z')
%         squeeze(world_to_camera_rotm_hist(50,:,:))
%          tr2eul(squeeze(world_to_camera_rotm_hist(50,:,:)))
%          size(permute(world_to_camera_rotm_hist,[2,3,1]))
euler_rotm_hist=tr2eul(permute(world_to_camera_rotm_hist,[2,3,1]));
subplot(2,3,4)
histogram(euler_rotm_hist(:,1),20)
title('rotation - X')
subplot(2,3,5)
histogram(euler_rotm_hist(:,2),20)
title('rotation - Y')
subplot(2,3,6)
histogram(euler_rotm_hist(:,3),20)
title('rotation - Z')

 sum((goodness_of_models_>0))

figure
subplot(2,3,1) ; title('x')
histogram(world_to_camera_trans_hist( (goodness_of_models_>0),1),100)
subplot(2,3,2) ; title('y')
histogram(world_to_camera_trans_hist( (goodness_of_models_>0),2),100)
subplot(2,3,3) ; title('z')
histogram(world_to_camera_trans_hist( (goodness_of_models_>0),3),100)
subplot(2,3,4); title('rotation - X')
histogram(euler_rotm_hist( (goodness_of_models_>0),1),100)
subplot(2,3,5); title('rotation - Y')
histogram(euler_rotm_hist( (goodness_of_models_>0),2),100)
subplot(2,3,6); title('rotation - Z')
histogram(euler_rotm_hist( (goodness_of_models_>0),3),100)


subplot(2,3,1) ; title('x')
subplot(2,3,2) ; title('y')
subplot(2,3,3) ; title('z')
subplot(2,3,4); title('rotation - X')
subplot(2,3,5); title('rotation - Y')
subplot(2,3,6); title('rotation - Z')

%%
%   GM = FITGMDIST(X,K) fits a Gaussian mixture distribution with K
%   components to the data in X.  X is an N-by-D matrix.  Rows of X
%   correspond to observations; columns correspond to variables.

% https://stackoverflow.com/questions/25085126/how-to-fit-multivariate-normal-distribution-to-data-in-matlab
GM1 = fitgmdist(world_to_camera_trans_hist,1)
GM2 = fitgmdist(world_to_camera_trans_hist,2)
GM3 = fitgmdist(world_to_camera_trans_hist,3)

GM1_x = fitgmdist(world_to_camera_trans_hist(:,1),1)
GM1_xy = fitgmdist(world_to_camera_trans_hist(:,1:2),1)
GM1_xz = fitgmdist([world_to_camera_trans_hist(:,1) world_to_camera_trans_hist(:,3) ],1)
% https://au.mathworks.com/help/stats/simulate-data-from-a-gaussian-mixture-model.html
GM %gmdistribution

% ??GM1.posterior ??
GM1.pdf(1,1,1)

[P,nlogl] = GM1.posterior(world_to_camera_trans_hist)
[P,nlogl] = posterior(GM1,world_to_camera_trans_hist)
[P,nlogl] = posterior(GM1,[1 1 1;2 2 2])
[P,nlogl] = posterior(GM2,world_to_camera_trans_hist)

gmPDF = @(x,y)pdf(GM1_xy,[x y]);
figure;
ezsurf(gmPDF,[-10 10],[-10 10])

gmPDF = @(x,y)pdf(GM1_xz,[x y]);
figure;
ezsurf(gmPDF,[-10 10],[-10 10])


%NORMFIT Parameter estimates and confidence intervals for normal data.
%   [MUHAT,SIGMAHAT] = NORMFIT(X) returns estimates of the parameters of
%   the normal distribution given the data in X.  MUHAT is an estimate of
%   the mean, and SIGMAHAT is an estimate of the standard deviation.
%   [MUHAT,SIGMAHAT,MUCI,SIGMACI] = NORMFIT(X) returns 95% confidence
%   intervals for the parameter estimates.
normfit  % fit a 1D Gaussian with confidences

 [MUHAT,SIGMAHAT,MUCI,SIGMACI] = normfit(world_to_camera_trans_hist(:,1))

%   Y = NORMPDF(X,MU,SIGMA) returns the pdf of the normal distribution with
%   mean MU and standard deviation SIGMA, evaluated at the values in X.
normpdf(world_to_camera_trans_hist(:,1),MUHAT,SIGMAHAT)
figure; hold on; grid on; 
plot( ...
    world_to_camera_trans_hist(:,1)' , ...
    normpdf(world_to_camera_trans_hist(:,1),MUHAT,SIGMAHAT)' , ...
    'rx' )
% plot(world_to_camera_trans_hist(:,1),'bs')
histogram(world_to_camera_trans_hist(:,1),20)

figure; hold on;
[N,EDGES] = histcounts(world_to_camera_trans_hist(:,1),20)
histogram(world_to_camera_trans_hist(:,1),EDGES)
plot( ...
    EDGES , ...
    normpdf(EDGES,MUHAT,SIGMAHAT)' .* max(N) , ...
    'rx' )
plot( ...
    EDGES , ...
    normpdf(EDGES,MUHAT,SIGMAHAT)' .* max(N) , ...
    'r' )
plot( ...
    EDGES , ...
    normpdf(EDGES,MUCI(1,1),SIGMAHAT)' .* max(N) , ...
    'bx' )
% plot( ...
%     EDGES , ...
%      [0 N ]' -  (normpdf(EDGES,MUHAT,SIGMAHAT)' .* max(N) )  , ...
%     'ks' )
% plot( ...
%     EDGES , ...
%      [0 N  ]' -  (normpdf(EDGES,MUHAT,SIGMAHAT)' .* max(N) )  , ...
%     'k' )


% PDF PDF for a Gaussian mixture distribution.
%    Y = PDF(OBJ,X) returns Y, a vector of length N containing the
%    probability density function (PDF) for the gmdistribution OBJ,
%    evaluated at the N-by-D data matrix X. Rows of X correspond to points,
%    columns correspond to variables. Y(I) is the PDF value of point I.
gmPDF = @(x,y)pdf(GM,[x y]);  
gmPDF = @(x,y)pdf(GM_xy,[x y]);  

figure

%   EZCONTOUR(FUN) plots the contour lines of FUN(X,Y) using CONTOUR. FUN
%   is plotted over the default domain -2*PI < X < 2*PI, -2*PI < Y < 2*PI.
ezcontour(gmPDF,[-10 10],[-10 10]);


%  https://au.mathworks.com/matlabcentral/answers/33666-gaussian-non-linear-fitting?s_tid=answers_rc1-2_p2_MLT

nonZeroBins = counts > 0;
lny = log(counts(nonZeroBins));
coeffs = polyfit(bins(nonZeroBins), lny, 2)
% Find the Gaussian parameters from the least squares parameters.
sigma = sqrt(-1/coeffs(1))
mu = coeffs(2) * sigma^2/2
amplitude = exp(coeffs(3) + mu^2/sigma^2)


%%

Mu = [1 2;-3 -5];
Sigma = cat(3,[2 0;0 .5],[1 0;0 1]);
P = ones(1,2)/2;
gm = gmdistribution(Mu,Sigma,P);

%Plot the contour of the pdf of the GMM.

gmPDF = @(x,y)pdf(gm,[x y]);

figure;
ezcontour(gmPDF,[-10 10],[-10 10]);
hold on
title('GMM - PDF Contours');
