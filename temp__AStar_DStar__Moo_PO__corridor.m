%addpath( '/mnt/nixbig/downloads/RobotciVisionToolbox-GIT/master/robotics-toolbox-matlab/astar' )
%%
%rmpath( '/mnt/nixbig/downloads/RobotciVisionToolbox-GIT/master/robotics-toolbox-matlab/astar' )
%%
%%
%addpath( '/mnt/nixbig/downloads/RobotciVisionToolbox-GIT/master/robotics-toolbox-matlab' )
%%
%rmpath( '/mnt/nixbig/downloads/RobotciVisionToolbox-GIT/master/robotics-toolbox-matlab' )

%%
addpath('/mnt/nixbig/ownCloud/project_code/3rd_party/robotics-toolbox-matlab/')
%%
addpath('/mnt/nixbig/ownCloud/project_code/')
%%

% 5m x 20m
map = zeros(200,50)  ;
         
         % observability costs 
         % start unobserved, and subtract the fields of view 
         cost_layer_1 = ones(size(map))  ;
         
         camera_coords = [ 35 ; 1 ]  ;
         
         FoV_half_angle_degs=45  ;
         vec_length=40 ;
         for vec_length_factor = 1:-0.1:0.3
             vec_length_iteration = ceil( vec_length*vec_length_factor )  ;
             for ray_angle_deg = -FoV_half_angle_degs:1:FoV_half_angle_degs
                 % ray_angle_deg = FoV_half_angle_degs  ;

                 ray_vec = [ zeros(vec_length_iteration,1)  (1:vec_length_iteration)' ]'  ;
                 ray_cells = ceil( rot2(  deg2rad(ray_angle_deg)  )*ray_vec )  ;
                 ray_cells = ray_cells + repmat( camera_coords , 1 , vec_length_iteration )  ;             
    %              cost_layer_1(ray_cells) = cost_layer_1(ray_cells) - 0.1  ;             
                %              figure_named('cost_layer_1') ; surf(cost_layer_1)             
                 for ii_ = 1:size(ray_cells,2)  % 4m range            
                     display(ii_)  ;
                     display(ray_cells(:,ii_))
                     %      cost_layer_1( ray_cells(1,ii_) , ray_cells(2,ii_) ) = cost_layer_1( ray_cells(1,ii_) , ray_cells(2,ii_) ) - 0.1  ;                  
                     cost_layer_1( ray_cells(1,ii_) , ray_cells(2,ii_) ) = 0.5*vec_length_factor  ;                  
                 end
             end
         end         
         
         camera_coords = [ 110 ; 1 ]  ;
         camera_angle_degs = 0  ;
         
         FoV_half_angle_degs=60  ;
         vec_length=35 ;
         for vec_length_factor = 1:-0.1:0.3
             vec_length_iteration = ceil( vec_length*vec_length_factor )  ;
             for ray_angle_deg = -FoV_half_angle_degs:1:FoV_half_angle_degs
                 % ray_angle_deg = FoV_half_angle_degs  ;

                 ray_vec = [ zeros(vec_length_iteration,1)  (1:vec_length_iteration)' ]'  ;
                 ray_cells = ceil( rot2( deg2rad(camera_angle_degs) ) * rot2(  deg2rad(ray_angle_deg)  )*ray_vec )  ;
                 ray_cells = ray_cells + repmat( camera_coords , 1 , vec_length_iteration )  ;             
    %              cost_layer_1(ray_cells) = cost_layer_1(ray_cells) - 0.1  ;             
                %              figure_named('cost_layer_1') ; surf(cost_layer_1)             
                 for ii_ = 1:size(ray_cells,2)  % 4m range            
                     display(ii_)  ;
                     display(ray_cells(:,ii_))
                     %      cost_layer_1( ray_cells(1,ii_) , ray_cells(2,ii_) ) = cost_layer_1( ray_cells(1,ii_) , ray_cells(2,ii_) ) - 0.1  ;                  
                     cost_layer_1( ray_cells(1,ii_) , ray_cells(2,ii_) ) = 0.5*vec_length_factor  ;                  
                 end
             end
         end
         
    

                 
         
        %          cost_layer_2 = zeros(100,120)  ;
        %          cost_layer_2(70:75,35:80) = 1;
        %          as.addCost(2,cost_layer_2);        % add 1st add'l cost layer L        
         
        figure_named('plan and move')
         start = [10,130];
         goal = [10;30];
        as = DstarMOO(map);    % create Navigation object
        as.addCost(1,cost_layer_1);        % add 1st add'l cost layer L
         as.plan(goal,2);       % setup costmap for specified goal
         as.path(start);        % plan solution path star-goal, animate
         P = as.path(start);    % plan solution path star-goal, return path
        
         
        figure_named('cost_layer_1') ; surf(cost_layer_1)
        hold on;  plot3_rows( [ P' ; ones(1,size(P,1)) ] ,'rx')
         
        
         % visualise the costmaps
         figure_named('costmaps')
         size(as.cost_get(2))
         subplot(2,2,1); surf(  as.cost_get(1)  ); title('as.cost\_get(1)')
         subplot(2,2,2); surf(  as.cost_get(2)  ); title('as.cost\_get(2)')
         subplot(2,2,3); surf(  as.cost_get(3)  ); title('as.cost\_get(3)')
%%

map = zeros(100,100)  ;
         goal = [50;30];
         goal = [40;90];
         start = [50;10];
         % as = AstarMOO(map);    % create Navigation object
         % as = AstarPO(map);    % create Navigation object    %  Error - Undefined function or variable 'paretofront'
         as = DstarMOO(map);    % create Navigation object
         % as = DstarPO(map);    % create Navigation object    %  Error - Undefined function or variable 'paretofront'
         cost_layer_1 = zeros(100,100)  ;         
         cost_layer_1(30:35,20:80) = 1;
         cost_layer_2 = zeros(100,100)  ;    
         cost_layer_2(50:55,40:80) = 1;
         as.addCost(1,cost_layer_1);        % add 1st add'l cost layer L
         as.addCost(2,cost_layer_2);        % add 1st add'l cost layer L
        
         as.plan(goal,2);       % setup costmap for specified goal
         as.path(start);        % plan solution path star-goal, animate
         P = as.path(start);    % plan solution path star-goal, return path
         
%%         
         
map = zeros(100,100)  ;
         goal = [50;30];
         start = [20;10];
         % as = AstarMOO(map);    % create Navigation object
         % as = AstarPO(map);    % create Navigation object
         as = DstarPO(map);    % create Navigation object    %  Error - Undefined function or variable 'paretofront'
         as.plan(goal,2);       % setup costmap for specified goal
         as.path(start);        % plan solution path star-goal, animate
         P = as.path(start);    % plan solution path star-goal, return path
         
         