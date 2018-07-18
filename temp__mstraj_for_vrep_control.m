Q0 = [0 0]
P = [ 1:10 ; repmat([2],1,10)   ]   
P(2,2:2:10) = P(2,2:2:10)*-1      
P=P'

figure  ;   hold on  ;   grid on  ;  axis equal   ;
%   plot2_rows(P','rx')   ; 

QDMAX = [5 5]  ;
TSEG = ones(1,10)  ;
DT = 1; % 0.1 ;
TACC = 2.5  ;

traj  =  mstraj(  P, QDMAX, [] , Q0, DT, TACC   ) 

plot2_rows(traj','bs')




 
