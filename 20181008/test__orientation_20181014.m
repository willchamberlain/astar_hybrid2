r = ...
[  0.99963176  0.02413066  0.01241195  0.0        
   0.01986322 -0.96232761  0.27116605  0.0        
   0.01848778 -0.27081965 -0.96245255  0.0        
   0.0          0.0          0.0          1.0        ]

%------------------------------------------------
%    base square to tabletop, turned to its right, my left as I'm looking at it : phone on side

r = [ ...
    0.8190923838537514, -0.0068729654001909845, -0.5736204573229465; -0.03384613060614793, -0.9987653193315987, -0.03636311790043954; -0.5726622967821177, 0.0491995858423368, -0.818313689606359
   ]

t = [ 0.8423627093773516,0.18382898985344362,0.9722376491498734 ]'

%------------------------------------------------
%    base square to phone, tilted up : phone on side

r = [ ...
        0.9997135615490332, -0.017607890223677706, 0.016209782750506264; -0.004103167931394519, -0.7933631627529051, -0.6087347993992411; 0.02357878003363344, 0.6084939228854949, -0.7932081611680172
    ]
t = [ 0.008858902276686717,-0.4655253997564073,1.0314585196690016 ]'


%% ------------------------------------------------
%    

s = '   base flat on tabletop, tilted up, turned 30 to its right/my left/anticlockwise : phone on side'

r = [ ...
        0.7232460016217678, 0.3505191079307496, -0.5950223324494279; 0.014261824191142664, -0.8690111908642961, -0.4945868483121635; -0.6904432065773889, 0.3492218566012866, -0.6335079110501175        
]
t = [ 0.7828768214805594,-0.38058222502794425,0.7597977564131851 ]'

pose_on_phone = rt2tr(r,t)

try_ = [  ...
0 0 1 
1 0 0
0 1 0
]


%------------------------------------------------
%    base flat on tabletop, tilted up, turned 30 to its right/my left/anticlockwise : 

s = '   base flat on tabletop, tilted up, turned 30 to its right/my left/anticlockwise : phone upright'

r = [ ...
       -0.05188573235537097, -0.8563110740148173, -0.513847463064222; -0.6588870059855257, -0.35730437205803056, 0.6619678988075223; -0.7504503875149753, 0.37291410569701766, -0.545673240731866
       ]
t = [  0.8979666873715132,-0.2665146882727282,0.7169414861665915  ]'

pose_on_phone = rt2tr(r,t)


%% ------------------------------------------------

pose_on_phone = rt2tr(r,t)


unit_x=[1 ; 0 ; 0 ; 1]

pose_on_phone*unit_x


figure_named('r'); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
draw_axes_direct(pose_on_phone(1:3,1:3),pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(pose_on_phone(1:3,1:3),pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)



%  this with the phone upright, then something else like transposing the columns
figure_named('try_ * r * inv(try_)'); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
a_ = try_ * r * try_'
draw_axes_direct(a_ ,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(a_ ,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

    %what's the y vector like --> how to re-arrange the columns/cells
    % this
    y_like = a_ * [ 0 ; 1 ; 0 ]
    p_handle = plot3( [y_like(1) 0] , [y_like(2) 0] , [y_like(3) 0] )
    x_like = a_ * [ 1 ; 0 ; 0 ]
    p_handle = plot3( [x_like(1) 0] , [x_like(2) 0] , [x_like(3) 0] )
    z_like = a_ * [ 0 ; 0 ; 1 ]
    p_handle = plot3( [z_like(1) 0] , [z_like(2) 0] , [z_like(3) 0] )
    
    a_ * eye(3)
    y_like2 = rotx(degtorad(-90)) * y_like
    y_2 = y_like2 
    p_handle = plot3( [y_like2(1) 0] , [y_like2(2) 0] , [y_like2(3) 0] , 'm')
    y_like2 = rotx(degtorad(-90)) * x_like
    x_2 = y_like2 
    p_handle = plot3( [y_like2(1) 0] , [y_like2(2) 0] , [y_like2(3) 0] , 'm')
    y_like2 = rotx(degtorad(-90)) * z_like
    z_2 = y_like2 
    p_handle = plot3( [y_like2(1) 0] , [y_like2(2) 0] , [y_like2(3) 0] , 'm')
    
    orient_2 = [ x_2 y_2 z_2 ]
    
    plot3(1,1,1,'mo','LineWidth',5)
    plot3(1+0.05,1+0.05,1+0.05,'mo','LineWidth',5)
    plot3(1-0.05,1-0.05,1-0.05,'mo','LineWidth',5)
    draw_axes_direct(orient_2 ,[1;1;1],'',1,'LineWidth',3)
    draw_axes_direct_c(orient_2 ,[1;1;1],'',0.8, 'k','LineWidth',3)
    
    
    %delete(p_handle)
%  this   





% this is the correct operation inv(B) * X * B 
%  still wrong outcome: need another B --> z vector of frame parallel to zy plane : want the y vector parallel to the xy plane
figure_named(strcat('inv(try_) * r * try_',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
a_ = try_' * r * try_
draw_axes_direct(a_ ,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(a_ ,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)


% original  :-  x vector of frame is parallel to the zx plane : want the y vector parallel to the xy plane
figure_named(strcat('r',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
rot_new = r ;
draw_axes_direct(rot_new,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(rot_new,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

% original rotated 90 around world Y  :-  x vector of frame is parallel to the zx plane : want the y vector parallel to the xy plane
figure_named(strcat('r',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
rot_new = roty(degtorad(90))*r ;
draw_axes_direct(rot_new,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(rot_new,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

%%


%  --> z parallel to zy plane
try_2 = [  try_(:,1) try_(:,3) try_(:,2)  ]
figure_named(strcat('1 3 2',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = try_2' * r * try_2  ;
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)


%  -->  z parallel to zx plane
try_2 = [  try_(:,2) try_(:,1) try_(:,3)  ]
figure_named(strcat('2 1 3',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = try_2' * r * try_2  ;
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

% y vector parallel to xy plane:  candidate
try_2 = [  try_(:,2) try_(:,3) try_(:,1)  ]
figure_named(strcat('2 3 1',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = try_2' * r * try_2  
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

% x vector parallel to zx plane
try_2 = [  try_(:,3) try_(:,1) try_(:,2)  ]
figure_named(strcat('3 1 2',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = try_2' * r * try_2  ;
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

% x vector almost parallel to xy plane  
try_2 = [  try_(:,3) try_(:,2) try_(:,1)  ]
figure_named(strcat('3 2 1',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = try_2' * r * try_2  ;
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)


%%  --------------------------------------------------------------

% y vector parallel to xy plane:  candidate
try_2 = [  try_(:,2) try_(:,3) try_(:,1)  ]
figure_named(strcat('2 3 1',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = try_2' * r * try_2  
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
patch( [-1 -1 2 2 ] , [ -1 2 2 -1 ] , ones(1,4)*pose_on_phone(3,4) , 'k' , 'FaceAlpha',0.2)
view(3)


% after this, still need to mirror through its zx plane/along its [0;1;0], then rotate 180 by world z
figure_named(strcat('2 3 1 -> ',s)); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
try_3 = roty(degtorad(180)) * try_2' * r * try_2  
draw_axes_direct(try_3,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(try_3,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
patch( [-1 -1 2 2 ] , [ -1 2 2 -1 ] , ones(1,4)*pose_on_phone(3,4) , 'k' , 'FaceAlpha',0.2)
view(3)


(roty(degtorad(180)) * try_2' * r * try_2 ) * [0;1;0]

