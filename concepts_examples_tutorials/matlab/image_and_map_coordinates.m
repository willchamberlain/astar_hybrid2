

f = figure;
f_sub_1 = subplot(2,2,1)
idisp(floorplan, 'axis',f_sub_1)


f_sub_2 = subplot(2,2,2)
idisp(floorplan,'ynormal',  'axis',f_sub_2) ; xlabel('ynormal:   x data:   u (pixels)'); ylabel('ynormal:   y data:   v (pixels)')


f_sub_2 = subplot(2,2,3)
s=surf(floorplan); s.EdgeColor='None'; xlabel('surf: data x') ;  ylabel('surf: data y')

