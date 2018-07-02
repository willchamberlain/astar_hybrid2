floorplan = zeros(300,300)  ;
idisp(floorplan)

floorplan(100,140:160)=1;
floorplan(100:160,140)=1;
floorplan(100:160,160)=1;
floorplan(160,140:160)=1;
floorplan(160:170,150)=1;

floorplan(100,190:240)=1;
floorplan(100:160,190)=1;
floorplan(100:160,240)=1;
floorplan(160,190:200)=1;
floorplan(120,200:230)=1;
floorplan(120:160,200)=1;
floorplan(120:160,230)=1;
floorplan(160,230:240)=1;

idisp(floorplan)

floorplan_vertices = zeros(300,300,3)  ;
%floorplan_vertices(:,:,3) = floorplan  ;
for yy_ = 2:299
    for xx_ = 2:299
        %  internal corner - _inside_ the corner
        floorplan_vertices(yy_,xx_,1) =  sum(sum(floorplan(yy_-1:yy_+1,xx_-1:xx_+1))) == 5  ;
        %  external corner - _on_ the line
        floorplan_vertices(yy_,xx_,2) =  floorplan(yy_,xx_) && sum(sum(floorplan(yy_-1:yy_+1,xx_-1:xx_+1))) ==3  &&  sum(sum(floorplan(yy_,xx_-1:xx_+1))) < 3 && sum(sum(floorplan(yy_-1:yy_+1,xx_))) < 3;        
        % ends of lines -  _?_
        floorplan_vertices(yy_,xx_,3) =  sum(sum(floorplan(yy_-1:yy_+1,xx_-1:xx_+1))) == 2  && ~floorplan_vertices(yy_,xx_,1) && ~floorplan_vertices(yy_,xx_,2) ;
    end
end
idisp(floorplan_vertices)
% floorplan_vertices2 = floorplan_vertices  ;
% floorplan_vertices2(:,:,1) = floorplan(:,:,1) > 0  & ~floorplan_vertices(:,:,1) & ~floorplan_vertices(:,:,2) & ~floorplan_vertices(:,:,3)  ;
% floorplan_vertices2(:,:,2) = floorplan(:,:,1) > 0  & ~floorplan_vertices(:,:,1) & ~floorplan_vertices(:,:,2) & ~floorplan_vertices(:,:,3)  ;
% floorplan_vertices2(:,:,3) = floorplan(:,:,1) > 0  & ~floorplan_vertices(:,:,1) & ~floorplan_vertices(:,:,2) & ~floorplan_vertices(:,:,3)  ;
idisp(floorplan_vertices)

% try to infill the internal corners of floorplans so that the distance function pushes the path out a bit 
%   - (if there's no interesting things there: kind of feels like a vector field or a potential field)
% Also: a navmesh would 
% Also: connecting 'entrances' / finding the internal and external hulls would do that.
% Also: kind of feel that internal corner regions are just different - 'internal' if no goal or sub-goal is in there, where a transition/door/entrance is a sub-goal
%  --> AFFORDANCE 
%    -->  elastic-band ??
%
% Flock solution?  Stick to wall (wall fixed dist to one side or other) OR  maximise dist to lateral neighbour --> state=wall_follow||spread_out
%
% nope 
% kernel=zeros(5,5)
% kernel(1:5,5)=1
% kernel(5,1:5)=1
% idisp(convn(floorplan,kernel,'same') ) ;

%  circle :  what points does a circle touch two walls which touch at a vertex (no door in thos walls) --> gap between circle and walls is 'dead' --> different
%  size circles up to infinite/line
imageSizeX = 640;  imageSizeY = 480;
[columnsInImage rowsInImage] = meshgrid(1:imageSizeX, 1:imageSizeY)  ;
% Next create the circle in the image.
centerX = 320;  centerY = 240;  radius = 100;
circlePixels = (rowsInImage - centerY).^2 ...
    + (columnsInImage - centerX).^2 <= radius.^2  ;
circlePixels = xor( circlePixels ,  (rowsInImage - centerY).^2 ...
    + (columnsInImage - centerX).^2 <= (radius-1).^2) ;  
% circlePixels is a 2D "logical" array. % Now, display it.
size(circlePixels)
surf(circlePixels*10)
image(circlePixels*255) ; axis equal

%  circle :  find internal corners and fill the triangle
% http://matlab.wikia.com/wiki/FAQ?title=FAQ&cb=7341#How_do_I_create_a_circle.3F


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

%%
idisp(path_10_20_radii_disp + repmat(floorplan,1,1,3) ) 
% idisp( imdilate(path_10_20_radii, ones(5)) )

[H, THETA, RHO] = hough(imdilate(path_10_20_radii, ones(6)), 'Theta', -90 : 5 : 89)  ;

neghbourhood_size = ceil(2*size(H)/50)  ;
mod(neghbourhood_size,2) ;
neghbourhood_size = neghbourhood_size + (1-mod(neghbourhood_size,2) )
% size(H)
% surf(H)
hough_peaks = houghpeaks(H,7, 'Threshold',0.3*max(H(:)), 'NHoodSize', ceil(2.24*size(H)/50)+1   )  ;
hough_lines = houghlines(imdilate(path_10_20_radii, ones(6)) , THETA , RHO , hough_peaks , 'FillGap',1,'MinLength',20 )

hold on
for k = 1:length(hough_lines)
   xy = [hough_lines(k).point1; hough_lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');
end



