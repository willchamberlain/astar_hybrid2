

f = figure;
f_sub_1 = subplot(1,2,1)
idisp(floorplan, 'axis',f_sub_1)

f_sub_2 = subplot(1,2,2)
idisp(floorplan,                    'axis',f_sub_1)
idisp(floorplan,'ynormal',  'axis',f_sub_2)



