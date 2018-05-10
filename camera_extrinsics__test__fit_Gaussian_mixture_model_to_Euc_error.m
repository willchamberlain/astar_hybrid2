mu1 = [1 2];
Sigma1 = [2 0; 0 0.5];
mu2 = [-3 -5];
Sigma2 = [1 0;0 1];
rng(1); % For reproducibility
X = [mvnrnd(mu1,Sigma1,1000);mvnrnd(mu2,Sigma2,1000)];

GMModel = fitgmdist(X,2);

figure;  plot(X)  ;  histogram(X,100)

GMModel = fitgmdist(eucidean_distance_error',2);
GMModel = fitgmdist(eucidean_distance_error',3);


big_error = eucidean_distance_error(eucidean_distance_error > 2)
GMModel = fitgmdist(big_error',1);

sorted = sort(eucidean_distance_error)
GMModel = fitgmdist(sorted',3);

% GMModel = fitgmdist(sorted', 2, 'start' , [ 10 990 ]);
GMModel = fitgmdist(sorted', 2, 'start' , [ repmat( [1] , [970 , 1] ) ; repmat( [2], [  size(sorted,2)-970, 1 ] ) ]);


