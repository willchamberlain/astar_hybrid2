

gauss_200_300 = gauss2d(zeros(200,300), [10,20] , [300/2,200/2])  ;

gauss_300_200 = gauss2d(zeros(300,200), [20,10] , [200/2, 300/2])  ;

gauss_300_200_padded = [ zeros(300,50) gauss_300_200 zeros(300,50) ]  ;
gauss_200_300_padded = [zeros(50,300) ; gauss_200_300 ; zeros(50,300)]  ;

gauss_mult = gauss_300_200*gauss_200_300 ;
gauss_padded_mult = gauss_300_200*gauss_200_300 ;


gauss_padded_sum = [ zeros(300,50) gauss_300_200 zeros(300,50) ] + [zeros(50,300) ; gauss_200_300 ; zeros(50,300)]  ;


figure; idisp(gauss_200_300)
figure; idisp(gauss_300_200)
figure; idisp(gauss_200_300_padded)
figure; idisp(gauss_300_200_padded)
figure; idisp(gauss_padded_mult)
figure; idisp(gauss_padded_sum)

figure; subplot(1,3,1); surf(gauss_200_300(1:200,1:200))
 subplot(1,3,2); surf(gauss_300_200(1:200,1:200))
 subplot(1,3,3); surf(gauss_mult(1:200,1:200))

% normpdf



gauss_300_200_long = gauss2d(zeros(300,200), [40,5] , [200/2,300/2])  ;
gauss_300_200_long = [zeros(300,50)  gauss_300_200_long  zeros(300,50)]  ;
f=figure; idisp(gauss_300_200_long)  ; f.Name='gauss_300_200_long'  ;
f=figure; idisp(gauss_300_200_padded)  ;f.Name='gauss_300_200_padded'  ;
f=figure; idisp(gauss_300_200_long*gauss_300_200_padded); f.Name='gauss_300_200_long*gauss_300_200_padded'  ;
f=figure; idisp(gauss_300_200_long*gauss_200_300_padded); f.Name='gauss_300_200_long*gauss_200_300_padded'  ;

gauss_300_200_padded_rotated = imrotate(gauss_300_200_padded,30)  ;
offset = (410-300)/2  ;
gauss_300_200_padded_rotated = gauss_300_200_padded_rotated(offset:offset+300-1,offset:offset+300-1)  ;

f=figure;  idisp(gauss_300_200_padded_rotated)  ; f.Name='gauss_300_200_padded_rotated'  ;
f=figure;  idisp(gauss_300_200_padded*gauss_300_200_padded_rotated)  ; f.Name='gauss_300_200_padded*gauss_300_200_padded_rotated'  ;
f=figure;  idisp(gauss_300_200_long*gauss_300_200_padded_rotated)  ; f.Name='gauss_300_200_long*gauss_300_200_padded_rotated'  ;


gauss_300_200_padded_rotatedneg = imrotate(gauss_300_200_padded,-30)  ;
size(gauss_300_200_padded_rotatedneg) ; offset = (410-300)/2  ;
gauss_300_200_padded_rotatedneg = gauss_300_200_padded_rotatedneg(offset:offset+300-1,offset:offset+300-1)  ;

f=figure;  idisp(gauss_300_200_padded_rotatedneg)  ; f.Name='gauss_300_200_padded_rotatedneg'  ;
f=figure;  idisp(gauss_300_200_padded_rotated*gauss_300_200_padded_rotatedneg)  ; f.Name='gauss_300_200_padded_rotated*gauss_300_200_padded_rotatedneg'  ;
f=figure;  idisp(gauss_300_200_padded_rotatedneg*gauss_300_200_padded_rotated)  ; f.Name='gauss_300_200_padded_rotatedneg*gauss_300_200_padded_rotated'  ;


%%---------------------------------------

mu = [0 0];
Sigma = [.25 .3; .3 1];
x1 = -3:.2:3; x2 = -3:.2:3;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,Sigma);
F = reshape(F,length(x2),length(x1));
f = figure;
surf(x1,x2,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis([-3 3 -3 3 0 .4])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density');
f.Name='Probability Density';


rotation_amt_ = rotz(degtorad(45))  ;
rotation_amt_ = rotation_amt_(1:2,1:2)  ;
sigma_rotated = rotation_amt_ * Sigma ;
x1 = -3:.2:3; x2 = -3:.2:3;
[X1,X2] = meshgrid(x1,x2);
F = mvnpdf([X1(:) X2(:)],mu,sigma_rotated);
F = reshape(F,length(x2),length(x1));
f = figure;
surf(x1,x2,F);
caxis([min(F(:))-.5*range(F(:)),max(F(:))]);
axis([-3 3 -3 3 0 .4])
xlabel('x1'); ylabel('x2'); zlabel('Probability Density rotated');
f.Name='Probability Density rotated';


