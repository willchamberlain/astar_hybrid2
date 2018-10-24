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

%  from android_core - ?main/java/william/chamberlain/androidvosopencvros/MainActivity.java:3459?
r = [ ...
       -0.05188573235537097, -0.8563110740148173, -0.513847463064222; -0.6588870059855257, -0.35730437205803056, 0.6619678988075223; -0.7504503875149753, 0.37291410569701766, -0.545673240731866
       ]
t = [  0.8979666873715132,-0.2665146882727282,0.7169414861665915  ]'

pose_on_phone = rt2tr(r,t)

try_ = [  ...
0 0 1 
1 0 0
0 1 0
]
%% ------------------------------------------------
%   Base flat on tabletop, vertical, yaw +20 
%   want: x_tag parallel xy_world , y_tag parallel xy_world , z_tag parallel to z_world 

s = '   Base flat on tabletop, vertical, yaw +20' 

%  from android_core
r = [ ...
    -0.017206208582719595, -0.9984396381082828, -0.05312471590897444; -0.9388669558110284, 0.034406250870812505, -0.3425566364663618; 0.3438499464480697, 0.0439829393653495, -0.9379939847208209
       ]
t = [  -0.41573823276137206,0.12028135881899521,1.0091068950063067  ]'

pose_on_phone = rt2tr(r,t)

try_ = [  ...
0 0 1 
1 0 0
0 1 0
]
%% ------------------------------------------------
%   Base flat on tabletop, vertical, yaw +20 
%   want: x_tag parallel xy_world , y_tag parallel xy_world , z_tag parallel to z_world 

s = '   Base flat on tabletop, vertical, yaw +20' 

%  from android_core
r = [ ...
    -0.024042460921678832, -0.9994326987736851, 0.02358475555901626; -0.9994398292296162, 0.023479865647488052, -0.023847927764319866; 0.02328063191379955, -0.02414490693966213, -0.9994373595411417 
    ]
t = [  -0.39539658600098126,0.12503715015778222,0.9852511701550373  ]'


%% --------------------------------------------------
% type=[target-to-camera] 
% feature_id=[1170]  
transl=[-0.0827418995079802,-0.03963235357805511,1.71069323173399]
t = transl'
transl_ttoc = transl'
rot=[0.815967908482565, -0.12553755751172946, 0.5643019528413643; -0.03820921668665518, -0.985713849643322, -0.16403738104936486; 0.5768331024540637, 0.11228770313498945, -0.8091075599930382 ] 
r=rot

pose_on_phone = rt2tr(r,t)
pose_on_phone = tr_invert(pose_on_phone)
[r,t] = tr2rt(pose_on_phone)

try_ = [  ...
0 0 1 
1 0 0
0 1 0
]

%% ------------------------------------------------
%  HI :  type=[target-to-camera] feature_id=[970]  
transl=[-0.6705331255631598,-0.03773382475002869,1.746576080392128]  
rot=[0.9955965228812935, 0.015929207850070588, -0.09237869864826899; -0.0024641465305731587, -0.9806692080036524, -0.1956574364939012; -0.09370961321312936, 0.1950234980989685, -0.9763116016829295 ] 
t = transl'
transl_ttoc = transl'
r=rot

pose_on_phone = rt2tr(r,t)
pose_on_phone = tr_invert(pose_on_phone)
[r,t] = tr2rt(pose_on_phone)

try_ = [  ...
0 0 1 
1 0 0
0 1 0
]


%% ------------------------------------------------
%  original pose 

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


%%  THIS
% Corrected pose 
%  this - with the phone upright, then something else like transposing the columns
figure_named('try_ * r * inv(try_)'); hold on; grid on; xlabel('x'); ylabel('y'); zlabel('z')
plot3(0,0,0,'bo')
plot3(2,2,2,'ro')
axis equal
a_ = try_ * r * try_'
draw_axes_direct(a_ ,pose_on_phone(1:3,4),'',1,'LineWidth',3)
draw_axes_direct_c(a_ ,pose_on_phone(1:3,4),'',0.8, 'k','LineWidth',3)
plot3_rows(pose_on_phone(1:3,4),'rx','LineWidth',5)
view(3)

    % this
    % CHECK --> Understand how the coordinate frame is composed of unit vectors
    %    y vector first: that is easier to check as it should be parallel to the xy_world plane 
    %    what's the y vector like --> how to re-arrange the columns/cells
    y_like = a_ * [ 0 ; 1 ; 0 ]
    x_like = a_ * [ 1 ; 0 ; 0 ]
    z_like = a_ * [ 0 ; 0 ; 1 ]
    p_handle = plot3( [y_like(1) 0] , [y_like(2) 0] , [y_like(3) 0] , 'g:')
    p_handle = plot3( [x_like(1) 0] , [x_like(2) 0] , [x_like(3) 0] , 'r:')
    p_handle = plot3( [z_like(1) 0] , [z_like(2) 0] , [z_like(3) 0] , 'b:')
    
    % check the values through the steps
    [ x_like y_like z_like ]
    a_
    a_ - [ x_like y_like z_like ]    
    
    % do the orientation
    y_2 = rotx(degtorad(-180)) * y_like
    x_2 = rotx(degtorad(-180)) * x_like
    z_2 = rotx(degtorad(-180)) * z_like
    p_handle = plot3( [y_2(1) 0] , [y_2(2) 0] , [y_2(3) 0] , 'g:','LineWidth',2)
    p_handle = plot3( [x_2(1) 0] , [x_2(2) 0] , [x_2(3) 0] , 'r:','LineWidth',2)
    p_handle = plot3( [z_2(1) 0] , [z_2(2) 0] , [z_2(3) 0] , 'b:','LineWidth',2)
    text(0,0,0,'right orientation, at origin')
       
    % check the values through the steps
    orient_2 = [ x_2 y_2 z_2 ]
                            rotx(degtorad(-90)) * [ x_like y_like z_like ]    
    orient_2  -      rotx(degtorad(-90)) * [ x_like y_like z_like ]  % = 0 : is the same
    orient_2  -      rotx(degtorad(-90)) * a_ % = 0 : is the same
    orient_2  -      rotx(degtorad(-90)) * ( try_ * r * try_' ) % = 0 : is the same
    
    transl_2 = try_*pose_on_phone(1:3,4)    % right
    transl_2 = try_*transl_ttoc
            rotx(degtorad(-90)) * try_ * pose_on_phone(1:3,4)   % wrong : vector doesn't need re-orienting
    
    
    % draw the corrected tag coordinate frame 
    draw_axes_direct(orient_2 ,transl_2,'',1,'LineWidth',3)
    draw_axes_direct_c(orient_2 ,transl_2,'',0.8, 'k','LineWidth',3)
    transl_ttoc
    % draw a pink blob on the axes origin to make it easier to check direction
    plot3_rows(transl_2,'mo','LineWidth',5)
    plot3_rows(transl_2+0.02,'mo','LineWidth',5)
    plot3_rows(transl_2-0.02,'mo','LineWidth',5)
    % label it
    text(transl_2(1),transl_2(2),transl_2(3),'right orientation, right position')
    
    
    
    %delete(p_handle)
%       END THIS   
%%

pose_on_phone_t = [-0.39539658600098126,0.12503715015778222,0.9852511701550373]'
try_*pose_on_phone_t
% - nope - just the rotation: vector to ROS frame is good with just the change-of-axis:   rotx(degtorad(-90)) * try_ * pose_on_phone_t

%%




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

