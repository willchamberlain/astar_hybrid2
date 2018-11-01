function map__ = oriented_cost_adjust_up(map_3_from_func)
    map_3_from_func_as_cost_factor = map_3_from_func*0.6;
    [ min(map_3_from_func_as_cost_factor(:))  max(map_3_from_func_as_cost_factor(:)) ] ;
    map_3_from_func_as_cost_factor = map_3_from_func_as_cost_factor + (1-max(map_3_from_func_as_cost_factor(:))) ; 
    [ min(map_3_from_func_as_cost_factor(:))  max(map_3_from_func_as_cost_factor(:)) ] ;
    map__ = map_3_from_func_as_cost_factor ;
end