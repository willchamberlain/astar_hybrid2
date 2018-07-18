% See http://www.it.uu.se/edu/course/homepage/grafik1/ht03/Lectures/LinePolygon/x_lines.htm
function points_on_line__ = geom__points_on_line(Xstart, Ystart, Xend, Yend)

%  line_spec = [ 100 140 ;
%     100.1 140 ] 
%  Xstart = line_spec(1,1)
%  Ystart = line_spec(1,2)
%  Xend = line_spec(2,1)
%  Yend = line_spec(2,2)

Xdifference = (Xend-Xstart)  ;
Ydifference = (Yend-Ystart)  ;

if Xdifference ~= 0
    delta_Y = Ydifference / Xdifference  ;
    if (Ydifference < 0 && delta_Y >0) || (Ydifference > 0 && delta_Y < 0)
        delta_Y = -delta_Y  ;
    end
    step = 1  ;
    if Xstart > Xend 
        step = -1  ;
    end
    num_x_steps =  size(Xstart : step : Xend  ,2)  ;
else
    delta_Y = 0 ;
    num_x_steps =  0 ;
end
if Ydifference ~= 0
    delta_X = Xdifference / Ydifference  ;
    if (Xdifference < 0 && delta_X >0) || (Xdifference > 0 && delta_X < 0)
        delta_X = -delta_X  ;
    end
    step = 1  ;
    if Ystart > Yend 
        step = -1  ;
    end
    num_y_steps =  size(Ystart : step : Yend  ,2)  ;
else
    delta_X = 0;
    num_y_steps =  0 ;
end


% figure; hold on; 
index = 1 ;
if 0 == Xdifference && 0 == Ydifference
        % plot([Xstart], [Ystart],'r.')  % plot_pixel(x, y)
        points_on_line__(index,:) = [Xstart,Ystart] ;
        index = index+1 ;
elseif Xdifference ~= 0
    y = Ystart  ;
    delta_Y = Ydifference / Xdifference  ;
    if (Ydifference < 0 && delta_Y >0) || (Ydifference > 0 && delta_Y < 0)
        delta_Y = -delta_Y  ;
    end
    step = 1  ;
    if Xstart > Xend 
        step = -1  ;
    end
    for x = Xstart : step : Xend
        %plot([x], [y],'r.')  % plot_pixel(x, y)
        points_on_line__(index,:) = [x,y] ;
        index = index+1  ;
        y = y + delta_Y  ;
    end
else % --> Ydifference ~= 0
    x = Xstart  ;
    delta_X = Xdifference / Ydifference  ;
    if (Xdifference < 0 && delta_X >0) || (Xdifference > 0 && delta_X < 0)
        delta_X = -delta_X  ;
    end
    step = 1  ;
    if Ystart > Yend 
        step = -1  ;
    end
    for y = Ystart : step : Yend
        %plot([x], [y],'r.')  % plot_pixel(x, y)
        points_on_line__(index,:) = [x,y] ;
        index = index+1 ;
        x = x + delta_X ;
    end    
end

