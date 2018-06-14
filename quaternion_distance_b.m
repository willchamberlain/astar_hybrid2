function var__ = quaternion_distance_b(q1_, q2_)
    display('NEEDS TESTING')
    var__ =  2*acos(abs(q1_.inner(q2_)))  ;
end