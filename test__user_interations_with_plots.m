function test__user_interations_with_plots()
    %  (2018a) https://au.mathworks.com/help/matlab/creating_plots/capturing-mouse-clicks.html
    f = figure;
    ax = axes;
    p = patch(rand(1,3),rand(1,3),'g');
    l = line([1 0],[0 1]);
    set(f,'ButtonDownFcn',@(~,~)figure_ButtonDownFcn(),...
       'HitTest','on')
    set(ax,'ButtonDownFcn',@(~,~)disp('axes'),...
       'HitTest','off')
    set(p,'ButtonDownFcn',@(~,~)disp('patch'),...
       'PickableParts','all','FaceColor','none')
    set(l,'ButtonDownFcn',@(~,~)lines_ButtonDownFcn(),...
       'HitTest','on')
   
    function asdfadfds = figure_ButtonDownFcn(obj_,event_)
        disp('figure: gca Currentpoint')
        gca_currentpoint = get(gca, 'Currentpoint')
        disp('figure: gco CurrentPoint')
            gco_handle = gco
            % set(gco)
            pt = get(gco_handle, 'CurrentPoint')
            disp('figure: gco CurrentPoint pt')
            disp(pt)
    end
   
    function asdfadfs = lines_ButtonDownFcn(obj_,event_)
        disp('line')                
        gca_currentpoint = get(gca, 'Currentpoint')
        x=get(gco,'Xdata')
        y=get(gco,'Ydata')
        pos=[x y]
          % convert to crosshairs
%          [xg,yg] = ginput(1)
%          pos_ginput=[xg yg]
         
        disp('line gcf_currentpoint')
        gcf_currentpoint = get(gcf, 'CurrentPoint')
        
        disp('line gca CurrentPoint')
            axes_handle = gca
            pt = get(axes_handle, 'CurrentPoint')
            
%         disp('gco CurrentPoint')
%             gco_handle = gco
%             set(gco)
%             pt = get(gco_handle, 'CurrentPoint')
            
            output_txt = {['X: ',num2str(pos(1),4)],...
            ['Y: ',num2str(pos(2),4)]};
        % If there is a Z-coordinate in the position, display it as well
        if length(pos) > 2
            output_txt{end+1} = ['Z: ',num2str(pos(3),4)];
        end
        disp(output_txt)                
    end

end