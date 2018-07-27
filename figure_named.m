function fig_handle__ = figure_named( figure_name_string_ , varargin)
    if  size(varargin,2) >= 1 && strcmpi(varargin{1} , 'default')
        fig_handle__ = figure('Name', figure_name_string_,varargin{2:end})  ;
        hold on; grid on; axis equal;
        xlabel('x'); ylabel('y'); zlabel('z');
    else
        fig_handle__ = figure('Name', figure_name_string_, varargin{:})  ;    
    end
end