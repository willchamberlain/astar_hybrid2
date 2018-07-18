floorplan = zeros(300,300)  ;
idisp(floorplan)

floorplan(100,140:160)=1;
floorplan(160,140:160)=1;
floorplan(100:160,140)=1;
floorplan(100:160,160)=1;
floorplan(160:170,150)=1;

floorplan(100,190:210)=1;
floorplan(160,190:210)=1;
floorplan(100:160,190)=1;
floorplan(100:160,210)=1;

idisp(floorplan)

% curved section: not working: use sin cos
% temp_ = zeros(100,100)  ;
% for ii_=50:90
%     for jj_=70:90
% %         temp_(jj_,ii_) = jj_^2 + ii_^2;
%             if jj_^2 + ii_^2 <= 10 + mean([50,90])^2+mean([70,90])^2  &&  jj_^2 + ii_^2 >= -10 + mean([50,90])^2+mean([70,90])^2
%                 temp_(jj_,ii_) = 1;
%             end
%     end
% end
% surf(temp_)

%  square kernel: not great
kernel = ones(7,7);
kernel(2:6,2:6)=0;
idisp(kernel)

% 

gaussian_ = gauss2d(zeros(19,19),8,[10,10]);
idisp(gaussian_)
gaussian_(gaussian_ >= gaussian_(7,1))=1;
idisp(gaussian_)
gaussian_(gaussian_ < gaussian_(7,1))=0;
idisp(gaussian_)
kernel = gaussian_ ;
C=convn(floorplan,kernel,'same') ;
idisp(C + floorplan)
% D=C ;
% D(D<7)=0;
% D(D>9)=0;
% idisp(D);
E=C;
for ii_ = 2:size(floorplan,2)-1
    for jj_ = 2:size(floorplan,1)-1
        local_ = C(jj_-1:jj_+1,ii_-1:ii_+1) ;
        if min(min(local_)) == 0 && C(jj_,ii_) > 0
            E(jj_,ii_)=1;
        else
            E(jj_,ii_)=0;            
        end
    end
end
figure; idisp(E)
figure; idisp(E+floorplan)


gaussian_2 = gauss2d(zeros(29,29),9,[15,15]);
figure; surf(gaussian_2)
gaussian_2(gaussian_2 >= gaussian_2(15,1))=1;
idisp(gaussian_2)
gaussian_2(gaussian_2 < gaussian_2(15,1))=0;
idisp(gaussian_2)
kernel = gaussian_2 ;
C_2=convn(floorplan,kernel,'same') ;
E_2=C_2;
for ii_ = 2:size(floorplan,2)-1
    for jj_ = 2:size(floorplan,1)-1
        local_ = C_2(jj_-1:jj_+1,ii_-1:ii_+1) ;
        if min(min(local_)) == 0 && C_2(jj_,ii_) > 0
            E_2(jj_,ii_)=1;
        else
            E_2(jj_,ii_)=0;            
        end
    end
end
figure; idisp(E_2)
figure; idisp(E+E_2+floorplan);
title('Not perfect: good enough for now.  Should use straight sections parallel to walls joined by const radius curves, or isolines on vector fields??')
title('Or roll a circle along the wall/around each tree, then path from envelope to envelope using rays to the outside of the envelope hull')

dist_im = bwdist(floorplan,'euclidean')  ;
idisp(dist_im)
surf(dist_im)
path_10_radii = zeros(300,300) + (dist_im >=9.5 & dist_im < 10.5) ;
idisp(path_10_radii)
path_10_20_radii = dist_im >=19.5 & dist_im < 20.5 ;
path_10_20_radii_disp = zeros(300,300,3) + cat(3,path_10_20_radii, zeros(300,300,2)) + cat(3,zeros(300,300,1), path_10_radii , zeros(300,300,1));
idisp(path_10_20_radii_disp + repmat(floorplan,1,1,3) ) 


hough path_10_20_radii


