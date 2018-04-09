function figure_handle = figure2(name)
figure_handle = figure('Name',name);   hold on; grid on; axis equal; xlabel('x'); ylabel('y');
end