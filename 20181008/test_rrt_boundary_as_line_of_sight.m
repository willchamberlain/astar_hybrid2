%{
https://au.mathworks.com/help/matlab/ref/boundary.html

Try:  treat the lines of sight to a volume as the boundary, with points defined by 
the 99%|95%|66% confidence bound, as appropriate. 
the closest faces of obstacles +floor and ceiling/sky -->  defining the interior of doorways etc as limits on the volume
--> can use boundary algorithms.
%}


P = gallery('uniformdata',30,3,5);

figure
plot3(P(:,1),P(:,2),P(:,3),'.','MarkerSize',10)
grid on
hold on 
axis equal

k = boundary(P);
hold on
trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor','red','FaceAlpha',0.1, 'Edgecolor',[0.2 0.2 0.2])



P = [  0, 1, 0  ;  0, 1, 2.3  ;  1,1,2.3 ;  1,1,0  ;
          0, 11, 0  ;  0, 11, 2.3  ;  1,11,2.3 ;  1,11,0  ;          
          1,15,0; 1,15,3 ;  5,18,0;5,18,3 ;  -5,18,0;-5,18,3 ;  -6,21,0;-6,21,3   ;  -5,24,0;-5,24,3 ;  -1,27,0;-1,27,3 ;
             1,27,0;1,27,3   ;  6,21,0;6,21,3   ;  5,24,0;5,24,3  ]
figure
plot3(P(:,1),P(:,2),P(:,3),'.','MarkerSize',10)
grid on
hold on 
axis equal

k = boundary(P,0.9);
hold on
trisurf(k,P(:,1),P(:,2),P(:,3),'Facecolor','red','FaceAlpha',0.1, 'Edgecolor',[0.2 0.2 0.2])


%  show the walls
patch(  [  0 0 -7 -7  ]  , ...
[ 11, 11, 11, 11 ]  , ...
[ 0  2.3  2.3 0  ] ,     ...
    'k', 'FaceAlpha',0.1)   

patch(  [  1 1 8 8   ]  , ...
[ 11, 11, 11, 11 ]  , ...
[ 0  2.3  2.3 0  ] ,     ...
    'k', 'FaceAlpha',0.1)   




