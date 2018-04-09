function inverted_tr = tr_invert( tr_ )
    R = tr_(1:3 , 1:3);
    T = tr_(1:3 , 4);
    inverted_tr =  vertcat(  horzcat( R', (-(R')) * T )  ,   [ 0 0 0 1]  );    
end