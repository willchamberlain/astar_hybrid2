addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%

covar = ...
    [   1       0.5
        0.5    1    ]  
    
covar_inv = inv(covar)    
    
command = ...
[   1 
    1   ]
covar*command



% state = [ x ; y ; theta ]  :  position = (x,y) in world coordinates, heading=orientation=theta in world coordinates

%{ 
motion model 

x(t) = x(t-1)  + time_step*
y(t) = y(t-1)  + time_step*
theta(t) = theta(t-1) + time_step*cmd_theta

%}

theta_deg=135
theta_rad = deg2rad(theta_deg)

vel = 10
time_step = 0.1

linspace( 1,1.2,10 )
command_list_pure = [ ... %  velocity ; theta 
    [   1.0   linspace( 1.0,1.2,150 ) linspace( 1.2,1.0,50 ) ]    % velocity = lateral change
    [   45/1   linspace( 45/1, (135-45)/1, 100)  linspace(  (135-45)/1, 45/1, 100)   ]  % angular vel = change angle from x-axis, in degrees 
]  ;
vel_range = [min(command_list_pure(1,:)),max(command_list_pure(1,:))] ;
vel_range_magnitude = vel_range(2)-vel_range(1);
theta_range = [min(command_list_pure(1,:)),max(command_list_pure(1,:))] ;
theta_range_magnitude = theta_range(2)-theta_range(1);
velocity_gaussian = [0 normrnd(0,  vel_range_magnitude , 1, 200)]  ;
theta_gaussian = [0 normrnd(0,theta_range_magnitude, 1, 200)]  ;

    command_list_deltas = [ ... %  approx. derivatives velocity ; theta  
        [   0.0   command_list_pure(1,2:end)-command_list_pure(1,1:end-1)   ]    
        [   0.0   command_list_pure(2,2:end)-command_list_pure(2,1:end-1)   ]   
    ]  ;
    command_list_noise = [... % noise proportional to the derivatives of the state variables
        command_list_deltas(1,:).*velocity_gaussian
        command_list_deltas(2,:).*theta_gaussian 
    ]  ;
command_list_noise = [...  % noise proportional to the state
    command_list_pure(1,:).*velocity_gaussian  % velocity - generate as independent samples from time-invariant Gaussian distribution scaled by current velocity
    command_list_pure(2,:).*theta_gaussian % angle from x-axis, in degrees - generate as independent samples from time-invariant Gaussian distribution scaled by current angle
]  ;

command_list = command_list_pure + command_list_noise  ;


[ x_pure , y_pure , theta_pure ] = temp__covariance_run_sim(command_list_pure, time_step)  ;
[ x , y , theta ] = temp__covariance_run_sim(command_list, time_step)  ;

time_string = time_string_for_figuretitle()  ;

figure_named(strcat('pure (no noise): ',time_string))  ;
subplot(1,2,1);  plot(x_pure,y_pure) ;  title('position x,y')
subplot(1,2,2);  hold off ; plot(theta_pure); title('angle')
for ii_ = 0:45:360
    hold on;  plot( [1, size(command_list,2)] , [ mod(deg2rad(ii_-1),2*pi), mod(deg2rad(ii_-1),2*pi) ] )
end

figure_named(strcat('with noise: ',time_string))  ;
subplot(2,2,1);  
    plot(x,y)  ;  hold on;    
    plot(x_pure,y_pure)  ;  
    title( 'position x,y' )
    legend( 'with noise' , 'without noise')
subplot(2,2,2)    
    title( 'effect of noise on posiiton' )
    plot(  x-x_pure,y-y_pure)  ;  
    legend('effect of noise' )
subplot(2,2,3)    
    plot(  x-x_pure)  ;  hold on;  
    plot( y-y_pure)  ;  
    title ('noise magnitude')
    legend('noise magnitude x','noise magnitude y')
    
    
figure_named('cumulative')  ;
subplot(1,2,2);  hold off ; 
    plot(theta);  hold on;  
    plot(theta_pure)
    plot(theta-theta_pure)
    %plot(cumsum(theta)) ; 
    %plot(cumsum(theta_pure))  ;
    %plot(cumsum(theta) - cumsum(theta_pure))  ;
    title( 'angle' )
    legend( 'with noise' , 'without noise' , 'effect of noise','')



for ii_ = 0:45:360
    hold on;  plot( [1, size(command_list,2)] , [ mod(deg2rad(ii_-1),2*pi), mod(deg2rad(ii_-1),2*pi) ] )
end




