function cost__ = path_planning__manhattan_distance(from_, to_)
    cost__ = sum( abs( int64([ to_ ]) - int64([ from_ ] )) );
end