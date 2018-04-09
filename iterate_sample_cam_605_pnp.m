estimates = zeros(10,3);
err = zeros(10,1);
for ii_ = 1:30 
    [aa bb cc] = func_cam_605_select_points();
    estimates(ii_,:) = [aa bb cc];
    err(ii_) = sqrt(sum(([0.887598151824310  -1.97885166499638  0.957895776523009] -  [aa bb cc] ).^2));
end

cam_605_good = [...
    0.887598151824310  -1.97885166499638  0.957895776523009  -0.0318571289684177  0.187864426800859  0.248082165295458  0.949814360672122
    ];
plot3(cam_605_good(:,1),cam_605_good(:,2),cam_605_good(:,3), 'ks');
grid on; axis equal; xlabel('x');ylabel('y');zlabel('z');

figure('Name','Estimates of Cam 605'); hold on; grid on; axis equal; xlabel('x');ylabel('y');zlabel('z');
plot3(estimates(:,1),estimates(:,2),estimates(:,3), 'rx');
plot3(cam_605_good(:,1),cam_605_good(:,2),cam_605_good(:,3), 'ks');




