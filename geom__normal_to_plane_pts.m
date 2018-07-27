function [ a_b_c__ , d__ ] = geom__normal_to_plane_pts( p1 , p2 , p3 )
    
    % These two vectors are in the plane
    v1 = p3 - p1  ;
    v2 = p2 - p1  ;

    % the cross product is a vector normal to the plane
    normal_vec = cross(v1, v2)  ;
    a_b_c__ = normal_vec  ;

    % This evaluates a * x3 + b * y3 + c * z3 which equals d
    d__ = dot(normal_vec, p3)  ;
    
end 