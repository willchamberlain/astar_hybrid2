function var__ = quaternion_distance_c(q1_, q2_)
    display('NEEDS TESTING')
    var__ =  acos(2*q1_.inner(q2_.inv)^2  - 1)  ;
end