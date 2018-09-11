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

map = zeros(100,120)  ;
         goal = [50;30];
         goal = [40;90];
         start = [120;10];
         % as = AstarMOO(map);    % create Navigation object
         % as = AstarPO(map);    % create Navigation object    %  Error - Undefined function or variable 'paretofront'
         as = DstarMOO(map);    % create Navigation object
         % as = DstarPO(map);    % create Navigation object    %  Error - Undefined function or variable 'paretofront'
         cost_layer_1 = zeros(100,120)  ;
         cost_layer_1(30:35, 30:120) = 1;
         as.addCost(1,cost_layer_1);        % add 1st add'l cost layer L
         
         cost_layer_2 = zeros(100,120)  ;
         cost_layer_2(70:75,35:80) = 1;
         as.addCost(2,cost_layer_2);        % add 1st add'l cost layer L        
         
        figure_named('plan and move')
         as.plan(goal,5);       % setup costmap for specified goal
         as.path(start);        % plan solution path star-goal, animate
         P = as.path(start);    % plan solution path star-goal, return path
         
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
         
         