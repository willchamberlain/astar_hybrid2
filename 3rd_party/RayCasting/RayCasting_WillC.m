function RayCasting_WillC

% RayTracing - Raytracing algorithm 4 demonstration purposes
% 
% Programmer:   Markus Petershofen
% Date:         01.09.2016

clc

%% Figure
InitialRoomSize = 15;
roomSize = InitialRoomSize;
myFig = figure('Color','w','Resize','on','MenuBar','none',...                               
    'NumberTitle','off','Name','Raycasting',...
    'units','normalized','Position',[0.2 0.2 0.6 0.6],...
    'WindowButtonUpFcn',@(s,e)myWindowButtonUpFcn,...
    'WindowButtonMotionFcn',@(s,e)myMotionFcn,...
    'WindowScrollWheelFcn',@myScrollWheelFcn);   
myAxes = axes('units','normalized','Position',[0 0 1 1],...
    'Parent',myFig,'XLim',[-roomSize*1.33 roomSize*3],'YLim',[-roomSize*1.2 roomSize*1.2],...
    'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
hold(myAxes,'on')
axis(myAxes,'off')

%% Walls
walls(1).initialShape = [roomSize roomSize -roomSize -roomSize roomSize; roomSize -roomSize -roomSize roomSize roomSize];
walls(2).initialShape = [28.5 28.5 12 12 28.5; 10 0 0 10 10]*roomSize/10;
walls(3).initialShape= [28.5 28.5 12 12 28.5; -1.5 -10 -10 -1.5 -1.5]*roomSize/10;
walls(4).initialShape = [26 26 14 14 26; 8.5 1.5 1.5 8.5 8.5]*roomSize/10;
for ii = 1:length(walls)
    walls(ii).shape = walls(ii).initialShape;
    walls(ii).plot = patch('XData',walls(ii).shape(1,:),'YData',walls(ii).shape(2,:),...
        'FaceColor','w','Parent',myAxes,...
        'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);
end

%% Car Object
car.coord = [0; 0];
car.Radius = 1;
car.shape = [sin(linspace(0,2*pi,100)); cos(linspace(0,2*pi,100))]*car.Radius;
car.color = [0.2 0.2 0.8; 0.8 0.8 0.1];

%% Rays
allRays.num = 50;
allRays.radius = 10;
allRays.ab = [2 2];
t = linspace(0,2*pi*(1-1/allRays.num),allRays.num);
x11 = zeros(allRays.num*2,3);
y11 = zeros(allRays.num*2,3);
x11(:,1) = [ sin(t)*allRays.radius*allRays.ab(1)+car.coord(1) , sin(t)*allRays.radius*allRays.ab(1)+car.coord(1)+1];
y11(:,1) = [ cos(t)*allRays.radius*allRays.ab(2)+car.coord(2) , cos(t)*allRays.radius*allRays.ab(2)+car.coord(2)+1 ] ;
x11(:,2) = [ sin(t)*car.Radius+car.coord(1) , sin(t)*car.Radius+car.coord(1)+1 ] ;
y11(:,2) = [ cos(t)*car.Radius+car.coord(2) , cos(t)*car.Radius+car.coord(2)+1 ] ;
x11(:,3) = NaN;
y11(:,3) = NaN;
x12 = reshape(x11',allRays.num*2*3,1);
y12 = reshape(y11',allRays.num*2*3,1);
allRays.plot = plot(x12,y12,'r','ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);

%% Other Objects
objs(1:10) = struct;
for ii = 1:length(objs)
    objs(ii).coord = [(1-2*rand)*(InitialRoomSize-2); (1-2*rand)*(InitialRoomSize-2)];
    objs(ii).shape = (rand(2,5)*2).*[1 1 -1 -1 1; 1 -1 -1 1 1];
    objs(ii).shape(:,5) = objs(ii).shape(:,1);
    objs(ii).color = [rand/1.5 0.6+rand*0.4 rand/1.1];
    objs(ii).plot = patch('XData',objs(ii).shape(1,:)+objs(ii).coord(1),'YData',objs(ii).shape(2,:)+objs(ii).coord(2),...
        'FaceColor',objs(ii).color,'Parent',myAxes,...
        'ButtonDownFcn',@(s,e)myButtonDownFcn('obj',ii));
end

%% Plot car
car.plot = patch('XData',car.shape(1,:)+car.coord(1),'YData',car.shape(2,:)+car.coord(2),...
    'FaceColor',car.color(1,:),'Parent',myAxes,'FaceAlpha',0.9,...
    'ButtonDownFcn',@(s,e)myButtonDownFcn('car',1));

%% Control Flags and Variables
myControl.buttonDownFlag = 0;
myControl.type = 'start';
myControl.index = 0;
myMotionFcn
myControl.type = '';

%% UI-Controls
scrollWheelButtonGroup = uibuttongroup('Position',[0.64 0.58 0.26 0.26],'Parent',myFig,'BackgroundColor','w','BorderType','none');
createUIcontrol('radiobutton',[0.05 0.775 0.9 0.15],'Radius',12,'k',scrollWheelButtonGroup,'on','','w');
createUIcontrol('radiobutton',[0.05 0.45 0.9 0.15],'No of Rays',12,'k',scrollWheelButtonGroup,'on','','w');
createUIcontrol('radiobutton',[0.05 0.1 0.9 0.15],'Room Size',12,'k',scrollWheelButtonGroup,'on','','w');
editBox(1) = createUIcontrol('edit',[0.79 0.775 0.1 0.05],num2str(allRays.radius),12,'k',myFig,'on',@(s,e)editchange(1),'w');
editBox(2) = createUIcontrol('edit',[0.79 0.685 0.1 0.05],num2str(allRays.num),12,'k',myFig,'on',@(s,e)editchange(2),'w');
editBox(3) = createUIcontrol('edit',[0.79 0.595 0.1 0.05],num2str(roomSize),12,'k',myFig,'on',@(s,e)editchange(3),'w');
myManual = sprintf(['Controls: \n Left Mouseclick: New Object \n '...
    'Left Mouseclick on Object: Move Object\n ' ...
    'Right Mouseclick on Object: Delete Object \n '...
    'Scrollwheel while clicked on Object: Change Size \n '...
    'Scrollwheel: Depends on Radiobutton']);
manualAxes = axes('units','normalized','Position',[0.6 0.15 0.3 0.3], 'Parent',myFig);
hold(manualAxes,'on')
axis(manualAxes,'off')
text(0,0.5,myManual,'Parent',manualAxes,'HorizontalAlignment','left','Interpreter','Latex','FontSize',10,...
        'ButtonDownFcn',@(s,e)wallPatchButtonDownFcn);

%% Nested Functions
    function myButtonDownFcn(type,index)
        switch myFig.SelectionType
            case {'normal','open'}
                myControl.buttonDownFlag = 1;
                myControl.type = type;
                myControl.index = index;
            case 'alt'
                if strcmp(type,'obj') && length(objs) > 1
                    delete(objs(index).plot)
                    objs(index) = [];
                    for kk = 1:length(objs)
                        set(objs(kk).plot,'ButtonDownFcn',@(s,e)myButtonDownFcn('obj',kk))
                    end
                    myControl.type = 'start';
                    myMotionFcn
                    myControl.type = '';
                end
        end
    end

    function myWindowButtonUpFcn
        myControl.buttonDownFlag = 0;
        myControl.type = '';
        myControl.index = 0;
    end

    function myMotionFcn
        if strcmp(myControl.type,'start') || (myControl.buttonDownFlag && ~isempty(myControl.type) && myControl.index > 0)
            C = myAxes.CurrentPoint(1,1:2);
            switch myControl.type
                case 'obj'
                    objs(myControl.index).coord = C';
                    set(objs(myControl.index).plot,'XData',objs(myControl.index).shape(1,:)+C(1),'YData',objs(myControl.index).shape(2,:)+C(2))
                case 'car'
                    car.coord = C';
                otherwise
            end
            % Create all rays
            x21 = zeros(allRays.num*2,3);
            y21 = zeros(allRays.num*2,3);
            x21(:,1) = [ sin(t)*allRays.radius*allRays.ab(1)+car.coord(1) , sin(t)*allRays.radius*allRays.ab(1)+car.coord(1)+1 ] ;
            y21(:,1) = [ cos(t)*allRays.radius*allRays.ab(2)+car.coord(2) , cos(t)*allRays.radius*allRays.ab(2)+car.coord(2)+1 ] ;
            x21(:,2) = [ sin(t)*car.Radius+car.coord(1) ,  sin(t)*car.Radius+car.coord(1)+1 ] ;
            y21(:,2) = [ cos(t)*car.Radius+car.coord(2) ,  cos(t)*car.Radius+car.coord(2)+1 ] ;
            x21(:,3) = NaN;
            y21(:,3) = NaN;
            x22 = reshape(x21',allRays.num*2*3,1);
            y22 = reshape(y21',allRays.num*2*3,1);
            % find intersections between rays and objects
            % only check for the objects inside the radius
            testfun = @(A,B) sqrt((A(1,:)-B(1,:)).^2+(A(2,:)-B(2,:)).^2);
            testDistances = reshape(bsxfun(testfun,[objs.shape]+reshape(repmat([objs.coord]',1,5)',2,5*length(objs)),repmat(car.coord,1,length(objs)*5)),5,length(objs));
            [~, myCol] = find(testDistances<2*allRays.radius);
            myTestObjs = objs(unique(myCol));
            for kk = 1:length(myTestObjs)
                [xi2, yi2, indexi2] = polyxpoly(x22, y22, myTestObjs(kk).shape(1,:)+myTestObjs(kk).coord(1), myTestObjs(kk).shape(2,:)+myTestObjs(kk).coord(2));
                if ~isempty(indexi2)
                    for mm = 1:length(xi2)
                        [shortestDistanceX, shortestDistanceY] = check4MultiHit(xi2,yi2,indexi2,mm);
                        x22(indexi2(mm,1)) = shortestDistanceX;
                        y22(indexi2(mm,1)) = shortestDistanceY;
                    end
                end
            end
            % find intersections between rays and walls
            for kk = 1:length(walls)
                [xi2, yi2, indexi2] = polyxpoly(x22, y22, walls(kk).shape(1,:), walls(kk).shape(2,:));
                if ~isempty(indexi2)
                    for mm = 1:length(xi2)
                        [shortestDistanceX, shortestDistanceY] = check4MultiHit(xi2,yi2,indexi2,mm);
                        x22(indexi2(mm,1)) = shortestDistanceX;
                        y22(indexi2(mm,1)) = shortestDistanceY;
                    end
                end
            end
            detectHit = 0;
            for kk = 1:length(myTestObjs)
                isIn = inpolygon(car.shape(1,:)+car.coord(1),car.shape(2,:)+car.coord(2),myTestObjs(kk).shape(1,:)+myTestObjs(kk).coord(1), myTestObjs(kk).shape(2,:)+myTestObjs(kk).coord(2));
                if any(isIn)
                    detectHit = 1;
                end
            end
            for kk = 1:length(walls)
                isIn = inpolygon(car.shape(1,:)+car.coord(1),car.shape(2,:)+car.coord(2),walls(kk).shape(1,:), walls(kk).shape(2,:));
                if numel(unique(isIn)) > 1
                    detectHit = 1;
                end
            end
            if detectHit
                curCarColor = car.color(2,:);
                curCarAlpha = 0.5;
            else
                curCarColor = car.color(1,:);
                curCarAlpha = 1;
            end
            set(car.plot,'XData',car.shape(1,:)+car.coord(1),'YData',car.shape(2,:)+car.coord(2),'FaceColor',curCarColor,'FaceAlpha',curCarAlpha)
            set(allRays.plot,'XData',x22,'YData',y22);
        end
    end

    function [shortestDistanceX,shortestDistanceY] = check4MultiHit(xi2,yi2,indexi2,mm)
        check4MultHit = find(indexi2(:,1)==indexi2(mm,1));
        % make sure that the closest intersection to the
        % car is considered (per ray)
        if length(check4MultHit) > 1
            if abs(xi2(check4MultHit(1))-car.coord(1)) > abs(xi2(check4MultHit(2))-car.coord(1))
                shortestDistanceX = xi2(check4MultHit(2));
            else
                shortestDistanceX = xi2(check4MultHit(1));
            end
            if abs(yi2(check4MultHit(1))-car.coord(2)) > abs(yi2(check4MultHit(2))-car.coord(2))
                shortestDistanceY = yi2(check4MultHit(2));
            else
                shortestDistanceY = yi2(check4MultHit(1));
            end
        else
            shortestDistanceX = xi2(check4MultHit(1));
            shortestDistanceY = yi2(check4MultHit(1));
        end
    end

    function editchange(editIndex)
        switch editIndex
            case 1
                allRays.radius = str2double(editBox(editIndex).String);
            case 2
                allRays.num = str2double(editBox(editIndex).String);
                t = linspace(0,2*pi*(1-1/allRays.num),allRays.num);
            case 3
                roomSize = str2double(editBox(editIndex).String);
                roomSize(roomSize<2) = 2;
                for kk = 1:length(walls)
                    walls(kk).shape = walls(kk).initialShape*roomSize/InitialRoomSize;
                    set(walls(kk).plot,'XData',walls(kk).shape(1,:),'YData',walls(kk).shape(2,:))
                end
                set(myAxes,'XLim',[-roomSize*1.33 roomSize*3],'YLim',[-roomSize*1.2 roomSize*1.2])
        end
        myControl.type = 'start';
        myMotionFcn
        myControl.type = '';
    end

    function wallPatchButtonDownFcn
        switch myFig.SelectionType
            case 'normal'
                newLength = length(objs)+1;
                objs(newLength).coord = myAxes.CurrentPoint(1,1:2)';
                objs(newLength).shape = (rand(2,5)*2).*[1 1 -1 -1 1; 1 -1 -1 1 1];
                objs(newLength).shape(:,5) = objs(newLength).shape(:,1);
                objs(newLength).color = [rand/1.5 1 rand/1.1];
                objs(newLength).plot = patch('XData',objs(newLength).shape(1,:)+objs(newLength).coord(1),'YData',objs(newLength).shape(2,:)+objs(newLength).coord(2),...
                    'FaceColor',objs(newLength).color,'Parent',myAxes,...
                    'ButtonDownFcn',@(s,e)myButtonDownFcn('obj',newLength));
                uistack(car.plot,'top')
        end
        myControl.type = 'start';
        myMotionFcn
        myControl.type = '';
    end
    
    function myScrollWheelFcn(~,evt)
        switch myControl.type
            case 'obj'
                if evt.VerticalScrollCount > 0
                    objs(myControl.index).shape = objs(myControl.index).shape/1.2;
                else
                    objs(myControl.index).shape = objs(myControl.index).shape*1.2;
                end
                set(objs(myControl.index).plot,'XData',objs(myControl.index).shape(1,:)+objs(myControl.index).coord(1),'YData',objs(myControl.index).shape(2,:)+objs(myControl.index).coord(2))
                myControl.type = 'start';
                myMotionFcn
                myControl.type = 'obj';
            case 'car'
                if evt.VerticalScrollCount > 0
                    car.Radius = car.Radius/1.2;
                else
                    car.Radius = car.Radius*1.2;
                end
                car.shape = [sin(linspace(0,2*pi,100)); cos(linspace(0,2*pi,100))]*car.Radius;
                set(car.plot,'XData',car.shape(1,:)+car.coord(1),'YData',car.shape(2,:)+car.coord(2))
                myControl.type = 'start';
                myMotionFcn
                myControl.type = 'car';
            otherwise
                allRadioButtons = get(scrollWheelButtonGroup,'Children');
                if allRadioButtons(2).Value == 1
                    allRays.num = allRays.num-evt.VerticalScrollCount;
                    allRays.num(allRays.num<1) = 1;
                    t = linspace(0,2*pi*(1-1/allRays.num),allRays.num);
                    editBox(2).String = num2str(allRays.num);
                elseif allRadioButtons(3).Value == 1
                    allRays.radius = allRays.radius-evt.VerticalScrollCount/5;
                    editBox(1).String = num2str(allRays.radius);
                elseif allRadioButtons(1).Value == 1
                    roomSize = roomSize-evt.VerticalScrollCount;
                    roomSize(roomSize<2) = 2;
                    for kk = 1:length(walls)
                        walls(kk).shape = walls(kk).initialShape*roomSize/InitialRoomSize;
                        set(walls(kk).plot,'XData',walls(kk).shape(1,:),'YData',walls(kk).shape(2,:))
                    end
                    set(myAxes,'XLim',[-roomSize*1.33 roomSize*3],'YLim',[-roomSize*1.2 roomSize*1.2])
                    editBox(3).String = num2str(roomSize);
                end
                myControl.type = 'start';
                myMotionFcn
                myControl.type = '';
        end
    end

    function UIvar = createUIcontrol(varType,varPos,varStr,varFontSize,varFColor,varParent,varVis,varCallback,varBColor)
        UIvar = uicontrol('Style',varType,...
            'units','normalized',...
            'Position',varPos,...
            'String',varStr,...
            'FontSize',varFontSize,...
            'FontName','Arial',...
            'FontUnits','normalized',...
            'BackgroundColor',varBColor,...
            'ForegroundColor',varFColor,...
            'Parent',varParent,...
            'Visible',varVis,...
            'Callback',varCallback,...
            'HorizontalAlignment','center');
    end
end