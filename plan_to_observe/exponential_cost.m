addpath '/mnt/nixbig/ownCloud/project_code/plan_to_observe/'
addpath '/mnt/nixbig/ownCloud/project_code/'
%%
%{
    Calculate exponential cost as approximation for feature detection measurement/sensor model.
%}

%%

floorplan_w = 7 % 7m wide
floorplan_l = 4 % 4m long
floorplan_scale = 0.1  % 10 cells/metre

cost_to_observe = floorplan_space ;

%%  1)  cost_to_observe as measurement model 
%{
Measurement model approx = /mnt/nixbig/ownCloud/project_code/temp__AStar_DStar__Moo_PO__corridor.m 
Illustrate field/cost vs distance
%}

dist = 0.0 : 0.1 : 8-0.1
y_1 = 1- (0.714285714285714*0.0005*exp(dist))   
figure; subplot(1,2,1); hold on; grid on; xlabel('distance (0.1m)'); ylabel('observability');
plot(y_1);  

dist = 0.0 : 0.1 : 8-0.1
y_1 = 1-(0.00275*exp(dist*(3/4) ))    % benefit in range 1-0
y_1 = (0.00275*exp(dist*(3/4) ))    %cost in range 0-1
subplot(1,2,2); hold on; grid on; xlabel('distance (0.1m)'); ylabel('observability');
plot(y_1);  

%%
%{
    Put the Robot_Target somewhere in the world
    Work out the field/cost to each cell
    Heatmap 
%}
figure; hold on; grid on; xlabel('distance (0.1m)'); ylabel('observability');
%    Put the Robot_Target somewhere in the world
robot_target_posn = [ ...
    round( randi( 10*floorplan_l) ), ...
    round( randi( 10*floorplan_w ) )  ];
% robot_target_posn = [   round(10*floorplan_l/2), round(10*floorplan_w/2) ]  ;
costmap_obs_dist = zeros(10*floorplan_l,10*floorplan_w)  ;
max_dist = sqrt( (10*floorplan_l)^2 + (10*floorplan_w)^2 ) ;
% CALCULATE COST
for length_ = 1.0 : 1 : 10*floorplan_l
    for width_ = 1.0 : 1 : 10*floorplan_w
        dist_cell_to_target = sqrt(   [ length_- robot_target_posn(1) ]^2  +   [ width_- robot_target_posn(2) ]^2   )  ;        
        costmap_obs_dist(length_, width_) =  (0.00275*exp(dist_cell_to_target*0.185*(3/4) ))   ;  % parameterisation : exp(d*x) : x spreads the function
        if costmap_obs_dist(length_, width_) > 1 
            costmap_obs_dist(length_, width_) = 1  ;
        elseif costmap_obs_dist(length_, width_) < 0  
            costmap_obs_dist(length_, width_) = 0  ;
        end
    end
end

% VISUALISE COST
figure
subplot(1,2 , 1);  colormap hot
surf( costmap_obs_dist )  
subplot(1,2 , 2);  colormap hot
surf( 1-costmap_obs_dist )  

