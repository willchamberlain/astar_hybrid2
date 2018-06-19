function time_string_for_figuretitle__ = time_string_for_figuretitle()
    clock_for_figuretitle=clock;     
    time_string_for_figuretitle__ = sprintf('_%d_%02d_%02d_%02d%02d%02d',[clock_for_figuretitle(1,1:5) round(clock_for_figuretitle(6))])  ;
end 