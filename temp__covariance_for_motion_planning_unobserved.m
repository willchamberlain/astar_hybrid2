
covariance_init=[ ...
    0.027323309 , -0.0016823198, 0, 0, 0, 0 
     -0.0016823198 , 0.0028801051, 0, 0, 0, 0
     0, 0, 0, 0, 0, 0
     0, 0, 0, 0, 0 ,0
     0, 0, 0, 0, 0 ,0
     0, 0, 0, 0, 0 , 0.00493741528
    ]   ;

covariance_x_y_theta_init =[ ...    
    0.027323309        , -0.0016823198, 0
    -0.0016823198     , 0.0028801051, 0
    0, 0 , 0.00493741528
    ]   ;

covariance_x_y_theta = covariance_x_y_theta_init ;


state_x_y_theta = [ ...
    0
    0
    0
    ]  ;

% motion_model

command_vel_twist = [ ...
    0.1
    +0.1
 ]


fig_posn = figure;  grid on; hold on ;  axis equal ;
fig_theta = figure;  grid on; hold on ;  axis equal ;
for ii_ = 1:200
    state_delta_x_y_theta = [ ...
        cos(state_x_y_theta(3)) * command_vel_twist(1)
        sin(state_x_y_theta(3)) * command_vel_twist(1)
        command_vel_twist(2)
        ]  ;
    
    state_updated_x_y_theta = state_x_y_theta + state_delta_x_y_theta/2  ;
    
%     covariance_update = 
    
    state_x_y_theta = state_updated_x_y_theta ;
    
    figure(fig_posn); plot(state_x_y_theta(1), state_x_y_theta(2), 'r.')
    figure(fig_theta); plot(state_x_y_theta(3), ii_, 'b.')    
end


%%

% pp127

% odometry noise 
alpha = 1

for alpha = [0.5 1 2]
V = alpha*diag([0.005, degtorad(0.3) ].^2)  ;
%The Toolbox Vehicle class simulates the bicycle model of Eq. 4.2 and the odometric
% configuration update Eq. 6.2. To use it we create a Vehicle object
veh = Vehicle(V)
% veh.L=0.1
% odo = veh.step(1, 0.3)
   
veh.add_driver( RandomPath(10) )
% where the argument to the RandomPath constructor specifies a working region that
% spans ±10 m in the x- and y-directions. We can display an animation of the robot with
% its driver by
%veh.run(1000)

veh.Fx( [0,0,0], [0.5, 0.1] )
veh.Fv( [0,0,0], [0.5, 0.1] )

% To simulate the vehicle and the EKF using the Toolbox we define the initial covariance
% to be quite small since, we assume, we have a good idea of where we are to begin with
P0 = diag([0.005, 0.005, 0.001].^2);
ekf = EKF(veh, V, P0);
ekf.run(1000);



if ~exist('fig_handle_path_','var') || ~ishandle(fig_handle_path_)
    fig_handle_path_ = figure('Name','Path and confidence bound ellipses'); grid on; 
else
    figure(fig_handle_path_) ;
end
veh.plot_xy()
hold on
ekf.plot_xy('r')

P700 = ekf.history(700).P

% The diagonal elements are the estimated variance associated with the states, that is σx2, σy2
% and σθ2 respectively. The standard deviation of the PDF associate with the x-coordinate is
standard_deviation_x = sqrt(P700(1,1))
variance_x = standard_deviation_x^2
conf_interval_x_95 = [ veh.x(1)-2*standard_deviation_x veh.x(1)+2*standard_deviation_x ]
% --> There is a 95% chance that the robot’s x-coordinate is within the ±2σ bound or ±1.37 m
% in this case. We can consider uncertainty for y and θ similarly.
% ekf.plot_ellipse([], 'g')

if ~exist('fig_handle_path_','var') || ~ishandle(fig_handle_path_) 
    fig_handle_path_ = figure('Name','Path and confidence bound ellipses'); grid on; 
else
    figure(fig_handle_path_) ;
end
ekf.plot_ellipse(20,'edgecolor','g') % every 20 timesteps 
ekf.plot_ellipse_bound(20,3, 'edgecolor','b') % every 20 timesteps 

if ~exist('fig_handle_total_uncertainty_','var')  || ~ishandle(fig_handle_total_uncertainty_)
    fig_handle_total_uncertainty_ = figure('Name','Total state uncertainy = sqrt(det(est_state))'); grid on; 
else
    figure(fig_handle_total_uncertainty_) ;
end
hold on; ekf.plot_P()


end







