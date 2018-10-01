function range__ =  small_large_range(some_, other_)
    if some_ > other_
        range__ = other_:some_ ;
    else
        range__ = some_:other_ ;
    end
end
