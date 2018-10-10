f1 = figure;
f1.ButtonDownFcn = @callBack_robot_controller;

hold on;
p1 = plot3(0,0,0,'bo')
axes = gca
axes.ButtonDownFcn = @callBack_robot_controller;
set(p1,'HitTest','off')
set(axes,'HitTest','off')
f1=gcf
figure_posn = get(f1,'Position')
width=figure_posn(3)
height=figure_posn(4)

my_appdata_struct.pioneer2_cmd_vel_pub = pioneer2_cmd_vel_pub
my_appdata_struct.pioneer2_cmd_vel_msg = pioneer2_cmd_vel_msg
figure__app_data_handles = guidata(gcf)
figure__app_data_handles = []
figure__app_data_handles.my_appdata_struct = my_appdata_struct  
guidata(gcf,figure__app_data_handles)
