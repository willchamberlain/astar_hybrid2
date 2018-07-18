function pp = temp__pure_pursuit
load_pp = 0;
    if load_pp == 0


        pp.STEP_TIME = 0.1;
        pp.SCALE_P = 1.0;
        pp.SCALE_V = 8.0;
        pp.WP_DIST = 20.0;         
        pp.FOLLOW_DIST = 40.0;     
        pp.X_OFF = 0;
        pp.Y_OFF = 0;
        pp.CRUMB_SZ = 1000;
        pp.NUM_WP = 5;
        pp.X = 1;
        pp.Y = 2;
        pp.DIM = 2;
        pp.valmax = 20/100; % <== Max steering angle 
        pp.isStart = false;
        pp.sim_counter = 0;

        pp.sim_t = 0.0;

        pp.sim_v = 0.0;    % velocity
        pp.sim_d = 0.0;    % steering

        % current point
        pp.sim_xp = 0.0;
        pp.sim_yp = 0.0;

        % current velocity
        pp.sim_xv = 0.0;
        pp.sim_yv = 0.0;

        % track points
        pp.steer_fx = 0.0;
        pp.steer_fy = 0.0;
        
        
    % double[][] way_pts = {{300.0,500.0},{100.0, 500.0},{100.0,100.0}, {500.0,100.0},{500.0, 500.0}};

    pp.way_pts = [300.0, 500.0 ; 100.0, 500.0 ; 120.0, 400.0 ; 90.0, 80.0 ; 100, 10 ; 500.0, 500.0];

    pp.cur_way_pt = 1; % current waypoint index


    % double[][] crumbs = new double[crumb_sz][dim];

    pp.crumbs = zeros(pp.CRUMB_SZ, pp.DIM);

    pp.crumb_idx = 1;

    pp = initSim(pp);

    pp.isStart = 1;

    

    loop.iter = 0;

    loop.maxiter = 1200;
        
        
        
    end
    
%% main loop here

    while 1 
        clf;  fig_pp = figure(1);   hold on;
        pp = draw(pp);
        title(sprintf('Pure pursuit control [%d]', loop.iter));
        hold off;   
%         % save figure every 10 times 
%         if rem(loop.iter , 10) == 0
%             set(fig_pp,'PaperPositionMode','auto')
%             print (fig_pp , '-dpng', ['pics/fig_pp_', num2str(loop.iter, '%03d'), '.png']) ;
%             % save current status 
%             save('pp_loop.mat', 'pp', 'loop');
%         end
        loop.iter = loop.iter + 1;
        % fprintf('[%d] \n', loop.iter);
        if loop.iter >= loop.maxiter
            break;
        end
    end
end

%% child functions 

% filledCircle([x y], 5, 100, 'b');
function h = filledCircle(centre_, radius, alpha, colour)
    r = radius ;
    d = r*2;
    x = centre_(1) ;
    y = centre_(2) ;
    px = x-r;
    py = y-r;
    h = rectangle('Position',[px py d d],'Curvature',[1,1], 'EdgeColor', colour, 'FaceColor', colour);
%     daspect([1,1,1])
end
function h = nofilledCircle(centre_, radius, alpha, colour)
    r = radius ;
    d = r*2;
    x = centre_(1) ;
    y = centre_(2) ;
    px = x-r;
    py = y-r;
    h = rectangle('Position',[px py d d],'Curvature',[1,1], 'EdgeColor', colour, 'FaceColor', 'none');
%     daspect([1,1,1])
end

function ret = random(fr, to)

ret = fr + (to-fr)*rand();
end

function pp = setPosition(pp, x, y)

pp.sim_xp = x;

pp.sim_yp = y;
end

function pp = setVelocity(pp, val)

pp.sim_v = val;
end

function pp = setSteering(pp, val)

pp.sim_d = val;
end

function pp = changeSteeringBy(pp, delta)

pp.sim_d = pp.sim_d + delta;
end

function pp = setFollow(pp, x, y)

pp.steer_fx = x;

pp.steer_fy = y;
end

function ret = getCurrentWaypoint(pp)

ret = pp.cur_way_pt;
end

function ret = getPreviousWaypoint(pp)

if pp.cur_way_pt == 1

    pwp = pp.NUM_WP;

else

    pwp = pp.cur_way_pt - 1;

end

ret =  pwp;
end

function pp = setup(pp)

pp = initsim(pp);
end

function pp = draw(pp)
    if (pp.isStart)
        pp = updateSim(pp);
        pp = updateSteering(pp, pp.sim_xp, pp.sim_yp, pp.sim_d);
        pp = updateWaypoint(pp, pp.sim_xp, pp.sim_yp);
        pp = updatePursuit(pp, pp.sim_xp, pp.sim_yp);
        pp = updateCrumbs(pp, pp.sim_xp, pp.sim_yp);
        pp = updateRover(pp);
    end 
    pp = drawCrumbs(pp);
    pp = drawWayPoints(pp);
    pp = drawRover(pp);
    pp = drawPursuit(pp);
    drawnow
end

function pp = initSim(pp) 
    pp.cur_way_pt = 1;
    pp = setFollow(pp, 280.0, 500);
    pp = setPosition(pp, 300.0, 400.0);
    pp = setVelocity(pp, 4);
    pp = setSteering(pp, 4);
    pp = updateRover(pp);
end

function pp = updateSim(pp)
pp.sim_counter = pp.sim_counter + 1;
pp.sim_t = pp.sim_t + pp.STEP_TIME;
 if rem(pp.sim_counter, 20) == 0
     % error every 20 times 
     pp.sim_d = pp.sim_d + random(-0.2, 0.2);
     pp.sim_xp = pp.sim_xp + random(-1.5, 1.5);
     pp.sim_yp = pp.sim_yp + random(-1.5, 1.5);
 end
end

function pp = updateSteering(pp, x, y, dir) 
fdx = (x-pp.steer_fx);
fdy = (y-pp.steer_fy);
fdist = calcDist(pp, fdx, fdy);
cdir = cos(dir);
sdir = sin(dir);
xprod = (sdir*fdx - cdir*fdy)/fdist;
% if xprod < -0.1 
%     val = -0.06;
% elseif xprod > 0.1 
%     val = +0.06;
% else
%     val = 0;
% end 
val = minmax_th(xprod, -pp.valmax, pp.valmax);
pp = changeSteeringBy(pp, val);
% fprintf('steering val: %.4f \n', val);
end


function pp = updateWaypoint(pp, x, y) 
cwpx = pp.way_pts(pp.cur_way_pt, pp.X);
cwpy = pp.way_pts(pp.cur_way_pt, pp.Y);
cdist = calcDist(pp, x-cwpx, y-cwpy);
if cdist < pp.WP_DIST 
    pp.cur_way_pt = pp.cur_way_pt + 1;
end
if pp.cur_way_pt > pp.NUM_WP
    pp.cur_way_pt = 1;
end
end


% main pp function 
function pp = updatePursuit(pp, x, y)
fdist = calcDist(pp, x-pp.steer_fx, y-pp.steer_fy);  
if (fdist < pp.FOLLOW_DIST)
    cwp = getCurrentWaypoint(pp);
    pwp = getPreviousWaypoint(pp);
    % fprintf('cur wp: %d prev wp: %d \n', cwp, pwp);
    wpx = pp.way_pts(cwp, pp.X) - pp.way_pts(pwp, pp.X);
    wpy = pp.way_pts(cwp, pp.Y) - pp.way_pts(pwp, pp.Y);
    wpdist = calcDist(pp, wpx, wpy);
    cdist = calcDist(pp, x-pp.way_pts(cwp, pp.X), y-pp.way_pts(cwp, pp.Y));
    wp_ratio = (wpdist - cdist + pp.FOLLOW_DIST)/wpdist;
    if wp_ratio > 1.0 
        wp_ratio = 1.0;
    end
    nx = pp.way_pts(pwp, pp.X) + wp_ratio*wpx;
    ny = pp.way_pts(pwp, pp.Y) + wp_ratio*wpy;
    pp = setFollow(pp, nx, ny);
end
end



function pp = updateCrumbs(pp, x, y)
if rem(pp.sim_counter, 3) == 0
    pp.crumbs(pp.crumb_idx, pp.X) = pp.sim_xp;
    pp.crumbs(pp.crumb_idx, pp.Y) = pp.sim_yp;
    pp.crumb_idx = pp.crumb_idx + 1;
    if (pp.crumb_idx >= pp.CRUMB_SZ)
        pp.crumb_idx = 1;
    end
end
end


% Update the position of the rover 

function pp = updateRover(pp) 
pp.sim_xv = pp.sim_v*cos(pp.sim_d);
pp.sim_yv = pp.sim_v*sin(pp.sim_d);
pp.sim_xp = pp.sim_xp + pp.sim_xv;
pp.sim_yp = pp.sim_yp + pp.sim_yv;
end


%%Draw stuffs 

function pp = drawRover(pp) 
x = calcDrawX(pp, pp.sim_xp);
y = calcDrawY(pp, pp.sim_yp);
xd = x + calcDrawV(pp, pp.sim_xv);
yd = y + calcDrawV(pp, pp.sim_yv);
filledCircle([x y], 10, 100, 'r');
plot([x xd], [y yd], 'b-');
text(50, 550, sprintf(' max steering: %.4f \n vel: %.4f', pp.valmax, pp.sim_v ) ...
    , 'BackgroundColor',[.3 .9 .7], 'FontSize', 7);
end

function pp = drawPursuit(pp)   
nx = calcDrawX(pp, pp.steer_fx);
ny = calcDrawY(pp, pp.steer_fy);
d = calcDraw(pp, pp.FOLLOW_DIST); 
filledCircle([nx ny], 5, 100, 'g');
nofilledCircle([nx ny], d, 100, 'g');
end



function pp = drawCrumbs(pp)   
% stroke(100);
% for i=0; i<pp.CRUMB_SZ; i++
% for i = 1:pp.CRUMB_SZ
%     plot(pp.crumbs(i, pp.X), pp.crumbs(i, pp.Y), 'o-');
% end
plot(pp.crumbs(1:pp.crumb_idx-1, pp.X), pp.crumbs(1:pp.crumb_idx-1, pp.Y), 'o-');
end


function pp = drawWayPoints(pp) 
fill(0, 0, 250);
% for (int i=0; i<NUM_WP; i++)
for i = 1:pp.NUM_WP
    x = calcDrawX(pp, pp.way_pts(i, pp.X));
    y = calcDrawY(pp, pp.way_pts(i, pp.Y));
    if i == pp.cur_way_pt
        d = calcDraw(pp, pp.WP_DIST);
        filledCircle([x y], 5, 100, 'b');
        nofilledCircle([x y], d, 100, 'b');
        text(x, y, sprintf(' [%d] <= current', i));
    else
        filledCircle([x y], 5, 100, 'b');
        text(x, y, sprintf(' [%d]', i));
    end 
end
end

function ret = calcDrawV(pp, d)

ret = d*pp.SCALE_V;
end
  

function ret = calcDraw(pp, d) 

ret = d*pp.SCALE_P;
end

function ret = calcDrawX(pp, x) 

ret = pp.X_OFF + calcDraw(pp, x);
end

function ret = calcDrawY(pp, y) 

ret = pp.Y_OFF + calcDraw(pp, y);
end

function ret = calcDist(pp, x, y) 

ret = sqrt((x*x + y*y));
end

function ret = minmax_th(x, xmin, xmax)

if x <= xmin

    ret = xmin;

elseif x >= xmax

    ret = xmax;

else

    ret = x;

end
end


