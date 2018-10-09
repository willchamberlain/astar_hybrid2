

gauss_20_30 = gauss2d(zeros(20,30), [1,2] , [15,10])  ;

gauss_30_20 = gauss2d(zeros(30,20), [2,1] , [10,15])  ;

gauss_mult = gauss_30_20*gauss_20_30 ;


figure; idisp(gauss_20_30)
figure; idisp(gauss_30_20)
figure; idisp(gauss_30_20*gauss_20_30)

figure; subplot(1,3,1); surf(gauss_20_30(1:20,1:20))
 subplot(1,3,2); surf(gauss_30_20(1:20,1:20))
 subplot(1,3,3); surf(gauss_mult(1:20,1:20))

normpdf