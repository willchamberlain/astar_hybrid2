addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%

% addpath( '/mnt/nixbig/downloads/matlab_DistBetween2Segment/DistBetween2Segment/' )
addpath(  '/mnt/nixbig/downloads/matlab_DistBetween2Segment/DistBetween2Segment/'  )

%{
varargout(1) = {outV}; % vector connecting the closest points
varargout(2) = {p2+sc*u}; % Closest point on object 1
varargout(3) = {p4+tc*v}; % Closest point on object 2
%}

%%

There is some bug. eg. 

a=[-0.744131766911425 -3.262495453200911 0.494264174913020]; 
b=[-1.455984810814321 -2.336190861135495 5.355871424229311]; 
c=[-0.099667764743042 -2.427139455191618 0.248449229997744]; 
d=[-4.464091294766189 -4.546406845194255 1.456966081453157];

[dist_1 vec_1 line1point_1 line2point_1] = DistBetween2Segment(a,b,c,d)

[dist_2 vec_2 line1point_2 line2point_2] = DistBetween2Segment(a,b,d,c)

I got 1.009752036238914 for DistBetween2Segment(a,b,c,d).

But if I switched point order, i got 3.914663312876543 for DistBetween2Segment(a,b,d,c)


figure_named('TEST line between 3D line segments');
hold on; grid on; 
plot3_rows( [a' b'])
plot3_rows( [c' d'])
plot3_rows(line1point_1','rx')
plot3_rows(line2point_1','bx')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'ms')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'m')
axis equal;

%%

a=[-0.744131766911425 -3.262495453200911  0]; 
b=[-1.455984810814321 -2.336190861135495  0]; 
c=[-0.099667764743042 -2.427139455191618  0]; 
d=[-4.464091294766189 -4.546406845194255  0];

[dist_1 vec_1 line1point_1 line2point_1] = DistBetween2Segment(a,b,c,d)

[dist_2 vec_2 line1point_2 line2point_2] = DistBetween2Segment(a,b,d,c)

figure_named('TEST line between 3D line segments in 2D plane - intersection');
hold on; grid on; 
plot3_rows( [a' b'])
plot3_rows( [c' d'])
plot3_rows(line1point_1','rx')
plot3_rows(line2point_1','bx')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'ms')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'m')
axis equal;


%%

a=[-0.744131766911425 -3.262495453200911  0]; 
b=[-1.455984810814321 -2.336190861135495  0]; 
c=[-0.099667764743042 -3.827139455191618  0]; 
d=[-4.464091294766189 -5.746406845194255  0];

[dist_1 vec_1 line1point_1 line2point_1] = DistBetween2Segment(a,b,c,d)

[dist_2 vec_2 line1point_2 line2point_2] = DistBetween2Segment(a,b,d,c)

figure_named('TEST line between 3D line segments in 2D plane - non-intersection');
hold on; grid on; 
plot3_rows( [a' b'] , 'g')
plot3_rows( [c' d'])
plot3_rows(line1point_1','rx')
plot3_rows(line2point_1','bx')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'ms')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'m')
axis equal;
%%

a=[-0.744131766911425 -3.262495453200911  0]; 
b=[-1.455984810814321 -2.336190861135495  0]; 
c=[-1.499667764743042 -3.027139455191618  0]; 
d=[-4.464091294766189 -5.746406845194255  0];

[dist_1 vec_1 line1point_1 line2point_1] = DistBetween2Segment(a,b,c,d)

[dist_2 vec_2 line1point_2 line2point_2] = DistBetween2Segment(b,a,d,c)

figure_named('TEST line between 3D line segments in 2D plane - non-intersection');
hold on; grid on; 
plot3_rows( [a' b'] , 'g')
plot3_rows( [c' d'])
plot3_rows(line1point_1','rx')
plot3_rows(line2point_1','bx')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'ms')
plot3_rows( [ line2point_1' line2point_1'+vec_1']   ,'m')
axis equal;


%%

% function [line1_exit__, line2_exit__, line_between_vec_, dist__] = shortestLineBetweenLineSegments( line1_pt1_, line1_pt2_, line2_pt1_, line2_pt2_)

a=[-0.744131766911425 -3.262495453200911 ]; 
b=[-1.455984810814321 -2.336190861135495 ]; 
c=[-1.499667764743042 -3.027139455191618 ]; 
d=[-4.464091294766189 -5.746406845194255 ];

%  [dist_1 vec_1 line1point_1 line2point_1] = DistBetween2Segment(a,b,c,d)

[line1_exit__, line2_exit__, line_between_veline2_exit__c_, dist__] = shortestLineBetweenLineSegments( a,b,c,d)

figure_named('TEST shortestLineBetweenLineSegments - line between 3D line segments in 2D plane - non-intersection');
hold on; grid on; 
plot3_rows( [ [a 0]' [b 0]'] , 'g')
plot3_rows( [ [c 0]' [d 0]'] , 'b')
plot3_rows(line1_exit__','rx')
plot3_rows(line2_exit__','rx')

( line2_exit__'+line_between_veline2_exit__c_'  )  - line1_exit__'

plot3_rows( [ line1_exit__' line2_exit__'+line_between_veline2_exit__c_']   ,'ms')
plot3_rows( [ line1_exit__' line2_exit__'+line_between_veline2_exit__c_']   ,'r')
plot3_rows( [ line1_exit__' line2_exit__']   ,'m')
axis equal;




