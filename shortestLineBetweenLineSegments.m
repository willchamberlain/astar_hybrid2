function [line1_exit__, line2_exit__, line2_exit_to_line1_exit_vec__, dist__] = shortestLineBetweenLineSegments( line1_pt1_, line1_pt2_, line2_pt1_, line2_pt2_)

    % if size(line1_pt1_) ~= [1 2] && size(line1_pt1_) ~= [2 1]
    % end
    % if size(line1_pt2_) ~= [1 2] && size(line1_pt2_) ~= [2 1]
    % end
    % if size(line2_pt1_) ~= [1 2] && size(line2_pt1_) ~= [2 1]
    % end
    % if size(line2_pt2_) ~= [1 2] && size(line2_pt2_) ~= [2 1]
    % end

a=[line1_pt1_(1)    line1_pt1_(2)   0]; 
b=[line1_pt2_(1)    line1_pt2_(2)   0]; 
c=[line2_pt1_(1)    line2_pt1_(2)   0]; 
d=[line2_pt2_(1)    line2_pt2_(2)   0]; 

[dist_1 , vec_1 , line1point_1 , line2point_1] = DistBetween2Segment(a,b,c,d)  ;

line1_exit__ = line1point_1  ;
line2_exit__ = line2point_1  ;
line2_exit_to_line1_exit_vec__ = vec_1  ;
dist__ = dist_1  ;


end