function fig_handle__ = figure_named( figure_name_string_ , varargin)
    fig_handle__ = figure('Name', figure_name_string_, varargin{:})  ;
end