function figure_handle = figure_named3(name)
figure_handle = figure('Name',name);   hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
end