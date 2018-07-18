function fig_3d_handle__ = setup_3d_figure(name_)

    fig_3d_handle__ = figure('Name',name_); axis equal; grid on; hold on;  xlabel('x'); ylabel('y'); zlabel('z');

end