%% 
%{
Treating FoV and unobserved as simple geometric.

If FoV edges intersect, aggregate the edges to define one composite FoV/covered region.
Within each covered region plan with AStar.
Between covered regions plan to use shortest path 
    NOTE:  around corners .
%}