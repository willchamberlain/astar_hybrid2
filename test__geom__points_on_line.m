
figure; hold on; grid on; axis equal; xlim([-12,12]); ylim([-12,12])
for ii_ = -100:100
    A=geom__points_on_line(0,0, ii_/10, 10)  ;
     plot_rows(A','Marker','.','Color',[ 1  0  abs((ii_+10)/255)/2 ])
    A=geom__points_on_line(0,0, ii_/10, -10)  ;
     plot_rows(A','Marker','.','Color',[ abs((ii_+10)/255)/2  0  1 ])
     pause(0.1)
end

for ii_ = -10:10
    A=geom__points_on_line(0,0, ii_/100, 10)  ;
     plot_rows(A','Marker','.','Color',[ 1  0  abs((ii_+10)/255)/2 ])
    A=geom__points_on_line(0,0, ii_/100, -10)  ;
     plot_rows(A','Marker','.','Color',[ abs((ii_+10)/255)/2  0  1 ])
     pause(0.1)
end
