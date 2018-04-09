function figure_handle = figure3(name)
figure_handle = figure('Name',name);   hold on; grid on; axis equal; xlabel('x'); ylabel('y'); zlabel('z');
end