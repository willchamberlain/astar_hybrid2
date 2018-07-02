function dummy_display_var_from_base_workspace(varname_)
    varvalue = evalin('base', 'varname_');
    display(strcat('varname_=',varname_)) 
    display(varvalue)
end