addpath('/mnt/nixbig/ownCloud/project_code/20181008/')
addpath('/mnt/nixbig/ownCloud/project_code/')

%%
%{
    Why :  Develop theory of information gain for active mobile vision sources & collaboration 
    between robots.
%}
%{
    Exp scenario :      Find cups and mugs in S11 : simplified version : 
        Start with a square world and some obstructions, so that we can 
        - test that it works
        - and then get results for the degree of clutter in the world
%}

%%

map_y_extent = 50  ;
map_x_extent = 50  ;
map_1 = ones(map_x_extent,map_y_extent)  ;
map_1_base_zero = zeros(map_x_extent,map_y_extent)  ;

%---

pillars{1} = Wall(inf(4,6) , [16,16]);
pillars{2} = Wall(inf(4,4) , [30,16]);
pillars{3} = Wall(inf(4,4) , [16,30]);
pillars{4} = Wall(inf(4,4) , [30,30]);

for ii_ = 1:size(pillars,2)
    map_1(...
        pillars{ii_}.location(1):pillars{ii_}.location(1)+size(pillars{ii_}.shape,1)-1, ...
        pillars{ii_}.location(2):pillars{ii_}.location(2)+size(pillars{ii_}.shape,2)-1) ...
    = pillars{ii_}.shape ;
    map_1_base_zero(...
        pillars{ii_}.location(1):pillars{ii_}.location(1)+size(pillars{ii_}.shape,1)-1, ...
        pillars{ii_}.location(2):pillars{ii_}.location(2)+size(pillars{ii_}.shape,2)-1) ...
    = pillars{ii_}.shape ;
end

idisp(map_1)

dist_func = bwdist(map_1_base_zero) ;
idisp(dist_func) 
%    cla
surf(dist_func) ; daspect([1 1 1])


robots{1} = SimpleRobot()  ;
robots{2} = SimpleRobot()  ;
robots{3} = SimpleRobot()  ;
robots{4} = SimpleRobot()  ;

for ii_ = 1:size(robots,2)
    ok = false;
    while ~ok
        x_ = randi(map_x_extent)  ;
        y_ = randi(map_x_extent)  ;
        if dist_func(x_,y_) >  6; ok = true ; 
            hold on;  grid on; plot3(x_,y_,dist_func(x_,y_), 'rx', 'LineWidth',5); 
            text(double(x_),double(y_),double(dist_func(x_,y_))+1.5, int2str(ii_), 'Color','r');       
        end 
    end
    ok2 = false;
    while ~ok2
        x2_ = max(round(randi(round(map_x_extent*2/3)) - map_x_extent/2  )  + x_, 3);
        y2_ = max(round(randi(round(map_x_extent*2/3)) - map_y_extent/2  ) + y_ , 3);
        if dist_func(x2_,y2_) > 6 ; ok2 = true ;         
            hold on;  grid on; plot3(x2_,y2_,dist_func(x2_,y2_), 'mx', 'LineWidth',5);  
            text(double(x2_),double(y2_),double(dist_func(x2_,y2_))+1.5, int2str(ii_), 'Color','m');  end ;
    end
end










