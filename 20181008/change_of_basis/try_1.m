%  vector

try_ = [  ...
0 0 1 
1 0 0
0 1 0
]

v1_boofcv = [  1.0 ; 0.2 ; 5.2  ] 

v1_xyz = try_ * v1_boofcv

% rotation 



%% ---------------------------------------------

BfromR = [   ...
    0 1 0
    0 0 1
    1 0 0
    ]

RfromB = BfromR'

RfromB - try_

yaw90B = roty(degtorad(90))

yaw90R =  RfromB * yaw90B * BfromR
              % = try_  * yaw90B * try_' 

rotz(degtorad(90))

yaw90R  - rotz(degtorad(90))


RfromB * yaw90B * BfromR