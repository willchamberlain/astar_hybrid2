function nr = findNorm(v1,v2)

nr = [0 0];
ang = atan2(v2(2)-v1(2),v2(1)-v1(1));

nr(1) = cos(ang - pi/2);
nr(2) = sin(ang - pi/2);

end