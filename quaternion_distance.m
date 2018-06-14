function var__ = quaternion_distance(q1_, q2_)
    display('NEEDS TESTING')
    quat_relative = q1_*q2_  ;
    var__ =  2*atan2(norm(quat_relative.v,2),quat_relative.s) - 2*pi  ;
end