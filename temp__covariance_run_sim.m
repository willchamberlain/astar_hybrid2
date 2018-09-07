function [ x , y , theta ]  =  temp__covariance_run_sim(command_list, time_step)

    x = zeros(1,size(command_list,2))  ;
    y = zeros(1,size(command_list,2))  ;
    theta = zeros(1,size(command_list,2))  ;

    x_initial = 0;
    y_initial = 0;
    theta_initial = deg2radwrapped( command_list(2,1) );
    x(1) = x_initial  ;
    y(1) = y_initial  ;
    theta(1) = theta_initial  ;

    for ii_ = 2:size(command_list,2)
        command = command_list(:,ii_)  ; %  velocity ; theta 
        %         command_prev = command_list(:,ii_-1)  ; %  velocity ; theta 
        %         state_diff_in_command = command-command_prev  ;

        %vel_in_time_step = vel*time_step
        vel_in_time_step = command(1)*time_step  ; % assume instantaneous change to commanded velocity:  manage translational acc and dec in settng up command_list

        theta_vel_in_time_step = command(2)*time_step  ; % assume instantaneous change to commanded theta:  manage rotational acc and dec in settng up command_list
        theta_vel_rad = deg2rad(theta_vel_in_time_step)  ;
        theta(ii_) = theta(ii_-1) + theta_vel_rad  ;

        %    cos(theta_rad) = x_delta / vel_in_time_step  

        x_delta = vel_in_time_step*cos(theta(ii_))  ;

        y_delta = vel_in_time_step*sin(theta(ii_))  ;

        %  theta_delta = assume instantaneous change to commanded theta

        %  update the state     
        x(ii_)  =  x(ii_-1)+x_delta  ;
        y(ii_)  =  y(ii_-1)+y_delta  ;
    end

    % 
    % subplot(1,2,1);  plot(x,y)
    % subplot(1,2,2);  hold off ; plot(theta); 
    % for ii_ = 0:45:360
    %     hold on;  plot( [1, size(command_list,2)] , [ mod(deg2rad(ii_-1),2*pi), mod(deg2rad(ii_-1),2*pi) ] )
    % end

end



