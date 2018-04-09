
BW1 = imread('/mnt/nixbig/ownCloud/project_code/map_S11_cutdown_by_hand_floorplan.png')  ; 
figure('Name','original'), imshow(BW1)

%%

idisp( kgauss(10) )
surf( kgauss(2.5,10) )
ksobel
BW3 = iconv(BW1,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
BW3 = iconv(BW3,kgauss(2.5,10.0));
idisp(BW3)



dx = DXform( double(255-BW1) );

goal = [10,10];
goal = [1; 1]
% dx.plan(goal)
% animate iterations of cost estimation
dx.plan(goal, 0.025);


for xx_ = 1:size(BW1,1)
    for yy_ = 1:size(BW1,2)
        done = false;
        if BW1(xx_,yy_) > 0
            for kk_ = 1:254
                lim_x_l = xx_-kk_;
                if lim_x_l < 1 ; lim_x_l =1; end
                lim_x_u = xx_+kk_;
                if lim_x_u > size(BW1,1) ; lim_x_u =size(BW1,1); end
                lim_y_l = yy_-kk_;
                if lim_y_l < 1 ; lim_y_l =1; end
                lim_y_u = yy_+kk_;
                if lim_y_u > size(BW1,2) ; lim_y_u =size(BW1,2); end
                if max(max(BW1(lim_x_l:lim_x_u,lim_y_l:lim_y_u))) >= 255 
                    BW3(xx_,yy_) = kk_ ;  
                    display(sprintf( 'continue %d %d', xx_, yy_) )
                    continue
                end
            end
        end
    end
end

hitormiss_kernel = [ ...
    NaN, 1, NaN;
         0, 1, 1;      
         0, 0, NaN;
    ];
BW3 =  hitormiss( BW1 ,  hitormiss_kernel);
idisp(BW1)
idisp(BW3)

%%
%   Robotics Vision and Control, 2nd ed.
%       5.2.3 Voronoi Roadmap Method
BW3 =  ithin(BW1);
idisp(BW3)

%%

bilinear_kernel = zeros(30,'double');
for ii_ = 1:floor(31/2)
    bilinear_kernel( ii_: size(bilinear_kernel,1)-ii_+1 , ii_: size(bilinear_kernel,2)-ii_+1) = 30 - ii_-1;
end
idisp(bilinear_kernel)

histogram(BW1)

BW3 = iconv(double(BW1),bilinear_kernel)   ; 
idisp(BW3)


imorph.c

bilinear_kernel = zeros(30,'double');
for ii_ = 1:floor(31/2)
    bilinear_kernel( ii_: size(bilinear_kernel,1)-ii_+1 , ii_: size(bilinear_kernel,2)-ii_+1) = ii_;
end
% idisp(bilinear_kernel)             histogram(bilinear_kernel)  
BW3 = iconv(255-double(BW1),bilinear_kernel)   ; 
idisp(BW3)
histogram(BW3)


BW3 = idilate(255-double(BW1),bilinear_kernel)   ; 
idisp(BW3)

BW1_inv_to_RVCtoolkit_image = 255-double(BW1);

BW_cummulative_neg = 0- BW1_inv_to_RVCtoolkit_image;
num_iterations=40
ii_ = 1
for ii_ = num_iterations:-1:1
    BW_next = idilate(BW1_inv_to_RVCtoolkit_image, ones(ii_+2) );
    BW_next = BW_next.*(num_iterations-ii_);
    BW_cummulative_neg = BW_cummulative_neg - BW_next;
end
idisp(BW_cummulative_neg)
histogram(BW_cummulative_neg)
histogram(BW_next)

%%

BW3 = 255-double(BW1);
idisp(BW3)
[D,IDX] = bwdist(BW3);
idisp(D)

%%
%  THIS BIT WORKS
%
%   [D,IDX] = BWDIST(BW) also computes the closest-pixel map in the form of
%   an index array, IDX. (The closest-pixel map is also called the feature
%   map, feature transform, or nearest-neighbor transform.) IDX has the
%   same size as BW and D. Each element of IDX contains the linear index of
%   the nearest nonzero pixel of BW.
%
%   [D,IDX] = BWDIST(BW,METHOD) lets you compute an alternate distance
%   transform, depending on the value of METHOD.  METHOD can be
%   'cityblock', 'chessboard', 'quasi-euclidean', or 'euclidean'.  METHOD
%   defaults to 'euclidean' if not specified.  METHOD may be
%   abbreviated.

EXP_CORRIDOR1 = iread('/mnt/nixbig/ownCloud/project_code/explicit_corridor_map.png', 'double');
paper_fig2a__Footprint = EXP_CORRIDOR1;
histogram(EXP_CORRIDOR1)  %  0-1
idisp(EXP_CORRIDOR1)
EXP_CORRIDOR_NEG = 1-EXP_CORRIDOR1(:,:,2)  ; 
idisp(EXP_CORRIDOR_NEG)  ;
[D,IDX] = bwdist(EXP_CORRIDOR_NEG)  ;
idisp(D)
% needs the border: the border is implicitly a wall in the paper
EXP_CORRIDOR_NEG = 1-EXP_CORRIDOR1(:,:,2)  ; 
EXP_CORRIDOR_NEG(:,1,:)=1;
EXP_CORRIDOR_NEG(1,:)=1;
EXP_CORRIDOR_NEG(end-1,:)=1;
EXP_CORRIDOR_NEG(:,end-1)=1;
idisp(EXP_CORRIDOR_NEG)
[D,IDX] = bwdist(EXP_CORRIDOR_NEG)  ;
idisp(IDX)  ; % looks somewhat like the framebuffer in the paper. ~= feature map, feature transform, or nearest-neighbor transform
idisp(D)  ;
paper_fig2b_ish__Framebuffer = EXP_CORRIDOR1;
MAP = D;
map_figure_handle=figure('Name','map'); idisp(MAP)  % WORKS
paper_fig2c__Zbuffer = D;
paper_figure_handle=figure('Name','from paper'); idisp(EXP_CORRIDOR1)  
addpath( '/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/vision/mex/' )
% EXP_CORRIDOR_NEG_THINNED = ithin(MAP, 0.01);
EXP_CORRIDOR_NEG_THINNED = ithin(MAP);
paper_fig2d__implicit_corridor_map_a = EXP_CORRIDOR_NEG_THINNED ;  % the lines, but not the nodes 
figure(map_figure_handle);   idisp(EXP_CORRIDOR_NEG_THINNED); hold on;     histogram(EXP_CORRIDOR_NEG_THINNED,20);   
%   NEXT    :   find the nodes in the thinned distance map / implicit corridor map 
%           5.2.3 Voronoi Roadmap Method  -  Robotics Vision and Control, 2nd ed.
addpath( '/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/vision/itriplepoint.m' )
addpath( '/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/vision/' )
EXP_CORRIDOR_NEG_THINNED_NODES =  itriplepoint( EXP_CORRIDOR_NEG_THINNED )  ;  % nodes in nodes in EXP_CORRIDOR_NEG_THINNED_NODES are at the junction of 3 lines
paper_fig2d__implicit_corridor_map_b  = EXP_CORRIDOR_NEG_THINNED_NODES  ;
figure(map_figure_handle); idisp(EXP_CORRIDOR_NEG_THINNED_NODES);


[nonzero_row, nonzero_col] = find(EXP_CORRIDOR_NEG_THINNED_NODES)  ;  % indices of nodes as  row, col of nodes in EXP_CORRIDOR_NEG_THINNED_NODES
nonzero_linear_indices = find(EXP_CORRIDOR_NEG_THINNED_NODES)  ;        % indices of nodes as linear indices in EXP_CORRIDOR_NEG_THINNED_NODES
EXP_CORRIDOR_NEG_THINNED_NODES(nonzero_linear_indices(1))

figure ; idisp(EXP_CORRIDOR_NEG_THINNED_NODES) ; hold on ; plot(nonzero_col,nonzero_row,'rx')


mod(nonzero_linear_indices, size(EXP_CORRIDOR_NEG_THINNED_NODES,1))  ;     % column    /   v
ceil( nonzero_linear_indices/size(EXP_CORRIDOR_NEG_THINNED_NODES,1) )  ;       %   row     /   u

% note: p3 of paper: doesn't mention the nodes in the algorithm 

idisp( EXP_CORRIDOR_NEG_THINNED )
histogram( EXP_CORRIDOR_NEG_THINNED )

histogram( double( EXP_CORRIDOR_NEG_THINNED_NODES ) )
    

figure;  idisp(EXP_CORRIDOR_NEG_THINNED - iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(3,'double') ) )
figure; histogram( EXP_CORRIDOR_NEG_THINNED , 20 )
figure; histogram( double( EXP_CORRIDOR_NEG_THINNED_NODES ) , 20 )
histogram ( iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(3,'double') )  , 20 )
histogram (EXP_CORRIDOR_NEG_THINNED - iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(3,'double') ) , 100 )
[ n , edges ] = histcounts (EXP_CORRIDOR_NEG_THINNED - iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(3,'double') )  )

im_ = EXP_CORRIDOR_NEG_THINNED - iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(3,'double') )  ; 
im_(im_<0) = 0;
idisp(im_)

% im_ = irotate(im_, 2*pi/4 );
% idisp(im_)
% histogram(kgauss(1), 1000)
% idisp(uint8(kgauss(1)>0.01))

node_removal_filter_width = 5;
im_(:,1:2)=0;
im_(1:2,:)=0;
im_(end-2:end-1,:)=0;
im_(:,end-2:end-1)=0;
im_ = idilate(im_ , ones(2) ); 
im_ = im_  -  iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(node_removal_filter_width,'double') )  ; 
im_(im_<0) = 0;
idisp(im_)

%% NEXT: try using  ilabel : iblobs.contains not doing it 
[L,MAXLABEL,PARENTS,CLASS,EDGE] = ilabel(im_);
idisp(L)

idisp(L==find(EDGE==1))
idisp(L==find(PARENTS==0))

ones(3).* [nonzero_col(1),nonzero_row(1)]
z = zeros(size(EXP_CORRIDOR_NEG_THINNED_NODES));
z(nonzero_col(1),nonzero_row(1))=1;
iconv(z,ones(node_removal_filter_width+1,'double') )

% %     eyeball to make sure it's the right 
        for jj_ = 1:size(nonzero_linear_indices,1)   
            z = zeros(size(EXP_CORRIDOR_NEG_THINNED_NODES));
%             z(nonzero_col(jj_),nonzero_row(jj_))=1;            
            z(nonzero_row(jj_),nonzero_col(jj_))=1;            
            idisp( 1-im_  +  iconv( z , ones( node_removal_filter_width+25 , 'double' ) ))
            pause
        end

idisp(1-im_)
hold on
for ii_ = 1:size(EDGE,1)
    if ii_ ~= find(EDGE==1)
        found_a_node = false;
        for jj_ = 1:size(nonzero_linear_indices,1)            
            z = zeros(size(EXP_CORRIDOR_NEG_THINNED_NODES));
%             z(nonzero_col(jj_),nonzero_row(jj_))=1;                   
            z(nonzero_row(jj_),nonzero_col(jj_))=1;          
            if  0 < sum(sum(    L==ii_ & iconv( z , ones( node_removal_filter_width+25 , 'double' ) )  ))
                found_a_node = true;
                display(sprintf( '\n labelled region  %d  touches node  %d  .\n' , ii_ , jj_ ))
                hold on
                plot(nonzero_col(jj_),nonzero_row(jj_),'rx')
                plot(nonzero_col(jj_),nonzero_row(jj_),'rs')
                text(nonzero_col(jj_)+10,nonzero_row(jj_)+10, sprintf('%d\_\_%d' , ii_ , jj_ ))
                pause
            end
        end
        if ~found_a_node
            fprintf('\nFound no node for region %d.\n',ii_);
        end
    else
        fprintf('\n%d touches the edge.\n',ii_)
    end
end




%%

contains_ = blob_1.contains( ...
    iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(5,'double') )  ...
)



blobs_as_RegionFeature = iblobs(im_, 'connect', 4,'boundary' , 1 , 'touch' , 0 );
% blobs_as_RegionFeature = iblobs(im_, 'connect', 4 ,'boundary' , 1);

figure;  hold on;
for ii_ = 1:size(blobs_as_RegionFeature,2)
    blob_1 = blobs_as_RegionFeature(ii_);
    if max(size(blob_1.edge)) <=2 
        fprintf('blob %d only has edge size %d,%d \n',ii_,size(blob_1.edge,1),size(blob_1.edge,2)) 
        if ~isempty(blob_1.edge) && min(blob_1.edge) > 0
            plot2_rows(  blob_1.edge , 'ks');        
        end
    else
        fprintf('blob %d edge size = %d,%d \n',ii_,size(blob_1.edge,1),size(blob_1.edge,2)) 
    end
    plot2_rows(  blob_1.edge );
%     plot2_rows(  blob_1.edge , 'k' );
    blob_1.plot()
    fprintf('%d',ii_)
    blob_1.contains([383,388])
    blob_1.contains([388,383])
%     pause
end

contains_ = blob_1.contains( ...
    iconv( double( EXP_CORRIDOR_NEG_THINNED_NODES ), ones(5,'double') )  ...
)

blob_1 = blobs_as_RegionFeature(1);
contains_ = blob_1.contains([383,388])
contains_ = blob_1.contains([388,383])
plot(388,383,'rx')
plot(383,388,'rd')
contains_ = blob_1.contains([385,388])
contains_ = blob_1.contains([388,385])
plot(388,385,'rx')
plot(385,388,'rd')
contains_ = blob_1.contains([387,388])
contains_ = blob_1.contains([388,387])
plot(388,387,'rx')
plot(387,388,'rd')
for ii_ = 383:400
    fprintf('%d',ii_)
    contains_ = blob_1.contains([ii_,388])
    plot(388,ii_,'rx')
    plot(ii_,388,'rd')
end
    
%   NEXT : find connections between the nodes --> graph 
self_mask = ones(3);
self_mask(2,2)=0;
dirs = [-1 0; -1 1; 0 1; 1 1; 1 0; 1 -1; 0 -1; -1 -1];
node_cols = mod(nonzero_linear_indices, size(EXP_CORRIDOR_NEG_THINNED_NODES,1))  ;     % column    /   v
node_rows = ceil( nonzero_linear_indices/size(EXP_CORRIDOR_NEG_THINNED_NODES,1) )  ;       %   row     /   u

node_3 = [ node_rows(3) , node_cols(3)  ]'  ; 
node_4 = [ node_rows(4) , node_cols(4)  ]'  ; 
node_3 = [ node_cols(3) , node_rows(3)  ]'  ; 
node_4 = [ node_cols(4) , node_rows(4)  ]'  ; 
node_3_connections = EXP_CORRIDOR_NEG_THINNED( node_3(1)-1:node_3(1)+1 , node_3(2)-1:node_3(2)+1) .* self_mask  ;
node_4_connections = EXP_CORRIDOR_NEG_THINNED( node_4(1)-1:node_4(1)+1 , node_4(2)-1:node_4(2)+1) .* self_mask  ;
node_3_connections_idx = find(node_3_connections)  ;
node_4_connections_idx = find(node_4_connections)  ;
for nc_a=1:size(node_3_connections_idx,1)         
    dirs(node_3_connections_idx(nc_a),:);
    for nc_b=1:size(node_4_connections_idx,1)         
        dirs(node_4_connections_idx(nc_b),:);
        % is connected - follow path 
        % ? iblobs ? 
        path_start_ = node_3 +  dirs(node_3_connections_idx(nc_a),:)'  ;
        path_end_    = node_4 +  dirs(node_4_connections_idx(nc_b),:)'  ; 
    end
end

    %{ 
    NEXT - start
    better: 
        for each node
            for each connector    
                if is not in a node_connection as   node_s, connector_at     (i.e. 3 , [228,81]  )
                follow the line until reach another node
                    record  node_connection node_s, connector_at  connected to  node_t, connector_at 
                            3 , [228,81]  , 4 , [252, 229]
                    record  node_connection  node_t, connector_at  connected to   node_s, connector_at
                            4 , [252, 229] , 3 , [228,81] 
    %}

%% find all the node connection points and record their locations so that we can check against them
addpath('/mnt/nixbig/ownCloud/project_code/')
squareGridUtils = SquareGridUtils
squareGridUtils.dirs            
            
            for ii_ = 1:size( nonzero_linear_indices ,1)   
                node_grid = [ node_cols(ii_) , node_rows(ii_)  ]'  ; 
                size(EXP_CORRIDOR_NEG_THINNED)
                node_connections = EXP_CORRIDOR_NEG_THINNED( node_grid(1)-1:node_grid(1)+1 , node_grid(2)-1:node_grid(2)+1) .* self_mask  ;                
                node_connections_idx = find(node_connections);
                for jj_ = 1:size(node_connections_idx,1)
                    node_connection_sq = NodeConnectorSquareGrid
                    node_connection_sq.node_a                               = ii_
                    node_connection_sq.node_a_grid                      = uint32(node_grid)
                    node_connection_sq.node_a_connector             = node_connections_idx(jj_)
                    node_connection_sq.node_a_connector_grid    = ...
                        node_connection_sq.node_a_grid ...
                        + squareGridUtils.dirs( node_connection_sq.node_a_connector , : )'
                    if ~exist( 'nodeConnector_list' , 'var' )
                        nodeConnector_list(1)=node_connection_sq;
                    else
                        nodeConnector_list(end+1)=node_connection_sq;
                    end
                end
            end
%%  for each  node connection point in turn, find which other node connection point(s) it connects to 
%{

    step traversal cost could be assumed to be 1 for all steps, or based on e.g. observability

    for each node_connection_sq in nodeConnector_list

        for each pixel around it
            if it == the node_connection_sq.node_a_grid
                skip
            if it == any other_  node_connection_sq.node_a_grid or node_connection_sq.node_a_connector_grid
                skip
            otherwise                
                if is on one of the line pixels
                    add pixel to open_set & mark as  -( step traversal cost )
                otherwise
                    leave as NaN 
    
           ((  open_set have value<0 , closed_set have value > 0 , unassessed have value NaN  ))
        for each pixel in open_set
            current cost = abs( marked value )  (( cost will always be greater than zero ; always takes time , even if not energy ))
            mark with current cost & add to closed set
            for each pixel around it
                if is on one of the line pixels
                    if it == the node_connection_sq.node_a_connector_grid
                         add pixel to closed_set & mark as  current cost + step traversal cost
                    if it == the node_connection_sq.node_a_grid
                         add pixel to closed_set & mark as  current cost + step traversal cost
                    if it == any other_  node_connection_sq.node_a_connector_grid
                        found a connection!
                        should be able to stop here, but may want to exhaust the  open set  in case branching is going on
                        add pixel to closed_set
                        mark as current cost + step traversal cost
                        record the connection
                    if it == any other_  node_connection_sq.node_a_grid
                         problem: should have hit  other_  node_connection_sq.node_a_connector_grid
                         add pixel to closed_set & mark as  current cost + step traversal cost
                    otherwise
                        add pixel to open_set & mark as  -(current cost + step traversal cost)
                otherwise
                    do nothing - leave as NaN


    MIGHT/COULD do this by convolution, keeping planes for 
        the thinned lines
        the nodes 
        the node connectors 
        the visited pixels - cummulative 
        the working pixels - initialise to the node connectors, valued to the node's number (e.g.  [ 0 3 0 ; 3 0 0 ; 0 0 3  ]  centred on node#3 ) 
    use the self-masked kernel , and conv on the working pixels plane
    check after each conv : 
        any   working pixels AND node connectors   are matches for a propogation reachig the location a connector
        the value of the working pixel gives the originating node 
%}

%{
NEXT - end
%}
pixel_now_ = path_start_;
pixel_last_(:,:,1) = pixel_now_;
pixel_last_(:,:,2) = pixel_now_;
pixel_last_(:,:,3) = pixel_now_;
while sum(pixel_now_ == path_end_) < 2 
    plot2_rows(pixel_now_ , 'cd')
    found = false  ;
    for ii_ = -1:1:1
        for jj_ = -1:1:1
            pixel_now_now_ = pixel_now_ + [ii_, jj_]' ;
            if ~empty(find(node_rows==pixel_now_now_(1) & node_cols==pixel_now_now_(2)))  % have matched to a node
                matching_node = find(node_rows==pixel_now_now_(1) & node_cols==pixel_now_now_(2));
                % get the connections somehow  NEXT
                for xx_= 1:size(node_3_connections_idx,1) 
                    if node_3 + dirs(xx_,:)'  == pixel_last_ (:,:,1)
                        %  matched :  NEXT: record it 
                        
                    end
                end
            end
            if ~(ii_ == 0 && jj_ == 0)  ...
                &&  sum(pixel_now_now_ ==  pixel_last_ (:,:,1)) < 2 ...
                &&  sum(pixel_now_now_ ==  pixel_last_ (:,:,2)) < 2 ...
                &&  sum(pixel_now_now_ ==  pixel_last_ (:,:,3)) < 2 ...
                && 1 == EXP_CORRIDOR_NEG_THINNED( pixel_now_(1)+ii_ , pixel_now_(2)+jj_ )
%                 &&  ii_ ~= pixel_last_(1) && jj_ ~= pixel_last_(2) ...
                        pixel_last_(:,:,3) = pixel_last_(:,:,2) ;
                        pixel_last_(:,:,2) = pixel_last_(:,:,1) ;
                        pixel_last_(:,:,1) = pixel_now_;
                        pixel_now_ = pixel_now_ + [ii_, jj_]'  ;     
                        plot(pixel_now_(2),pixel_now_(1), 'bs')                                   
                        found = true  ;
                        drawnow
                        fprintf('-- pixel_now_=[ %d ; %d ]\n', pixel_now_(1), pixel_now_(2))
            else
                fprintf('ii_=%d , jj_=%d , pixel=%d\n' , ii_ , jj_ , EXP_CORRIDOR_NEG_THINNED(pixel_now_(1)+ii_, pixel_now_(2)+jj_) )
            end
        end
    end
    if ~found 
        display('not found')
        break
    end
end



%   NEXT    :   find a path along the implicit corridor line from start to goal - start at the nearest point on the line to the start, end on the nearest point of the line to the goal 
[ D ,IDX ] = bwdist(EXP_CORRIDOR_NEG_THINNED) ; % distance to the line, and nearest point on the line for point on the map
figure;     idisp(D)   ;      histogram(D,100)   ;     histogram(IDX)    ;
start_ = [266, 213]'  ;
end_ = [400, 418 ]'  ;
hold on
plot(start_(1,1), start_(2,1), 'rx')
plot(end_(1,1), end_(2,1), 'bx')
plot( ceil(IDX(start_(1,1), start_(2,1)) / size(D,2))  ,  mod( IDX(start_(1,1), start_(2,1)) , size(D,2) )  ,   'md')
plot( ceil(IDX(end_(1,1), end_(2,1)) / size(D,2))  ,  mod( IDX(end_(1,1), end_(2,1)) , size(D,2) )  ,   'md')
IDX(end_(1,1), end_(2,1))

size(IDX)
size(EXP_CORRIDOR_NEG_THINNED)

ceil(1000/700)
mod(1000,700)
    %   -   meh     -   try again later 
    
%   NEXT    :   sample along the path in   EXP_CORRIDOR_NEG_THINNED

% start at 
path_start_ = [ 252 , 228  ]'  ;
path_end_ = [ 383 , 379 ]'   ;
        addpath('/mnt/nixbig/ownCloud/project_code')
hold on ;   plot2_rows(path_start_, 'bx')
pixel_now_ = path_start_;
pixel_last_(:,:,1) = pixel_now_;
pixel_last_(:,:,2) = pixel_now_;
pixel_last_(:,:,3) = pixel_now_;
while sum(pixel_now_ == path_end_) < 2 
    plot2_rows(pixel_now_ , 'cd')
    found = false  ;
    for ii_ = -1:1:1
        for jj_ = -1:1:1
            pixel_now_now_ = pixel_now_ + [ii_, jj_]' ;
            if ~(ii_ == 0 && jj_ == 0)  ...
                &&  sum(pixel_now_now_ ==  pixel_last_ (:,:,1)) < 2 ...
                &&  sum(pixel_now_now_ ==  pixel_last_ (:,:,2)) < 2 ...
                &&  sum(pixel_now_now_ ==  pixel_last_ (:,:,3)) < 2 ...
                && 1 == EXP_CORRIDOR_NEG_THINNED( pixel_now_(1)+ii_ , pixel_now_(2)+jj_ )
%                 &&  ii_ ~= pixel_last_(1) && jj_ ~= pixel_last_(2) ...
                        pixel_last_(:,:,3) = pixel_last_(:,:,2) ;
                        pixel_last_(:,:,2) = pixel_last_(:,:,1) ;
                        pixel_last_(:,:,1) = pixel_now_;
                        pixel_now_ = pixel_now_ + [ii_, jj_]'  ;     
                        plot(pixel_now_(2),pixel_now_(1), 'bs')                                   
                        found = true  ;
                        drawnow
                        fprintf('-- pixel_now_=[ %d ; %d ]\n', pixel_now_(1), pixel_now_(2))
            else
                fprintf('ii_=%d , jj_=%d , pixel=%d\n' , ii_ , jj_ , EXP_CORRIDOR_NEG_THINNED(pixel_now_(1)+ii_, pixel_now_(2)+jj_) )
            end
        end
    end
    if ~found 
        display('not found')
        break
    end
end

plot(path_start_(1),path_start_(2),'ws')
plot(path_end_(1),path_end_(2),'ws')

EXP_CORRIDOR_NEG_THINNED( pixel_now_(1)-1:pixel_now_(1)+1 , pixel_now_(2)-1:pixel_now_(2)+1 )
EXP_CORRIDOR_NEG_THINNED( pixel_now_(1)-5:pixel_now_(1)+5 , pixel_now_(2)-5:pixel_now_(2)+5 )
EXP_CORRIDOR_NEG_THINNED( path_start_(1)-5:path_start_(1)+5 , path_start_(2)-5:path_start_(2)+5 )
EXP_CORRIDOR_NEG_THINNED( path_start_(2)-5:path_start_(2)+5 , path_start_(1)-5:path_start_(1)+5 )
EXP_CORRIDOR_NEG_THINNED( path_end_(1)-5:path_end_(1)+5 , path_end_(2)-5:path_end_(2)+5 )
figure; histogram(EXP_CORRIDOR_NEG_THINNED, 50)

sum(sum(EXP_CORRIDOR_NEG_THINNED==0))

EXP_CORRIDOR_NEG_THINNED(245,228)
EXP_CORRIDOR_NEG_THINNED( 383 , 379 )

ii_ = 0 ; jj_ = 0 ;
EXP_CORRIDOR_NEG_THINNED(pixel_now_(1)+ii_, pixel_now_(2)+jj_) 
EXP_CORRIDOR_NEG_THINNED(pixel_now_(1)-1:pixel_now_(1)+1,  pixel_now_(2)-1:pixel_now_(2)+1) 

%%
%       ??
% Notes::
% - The output image could be thresholded to determine color similarity.
% - Note that Euclidean distance in the rg-chromaticity space does not 
%   correspond well with human perception of color differences.  Perceptually
%   uniform spaces such as Lab remedy this problem.
colordistance
%   equivalent on LAB good for TEXTURE / HSV binning / colour consistency
%   histograms in areas/for objects used by designed for humans ??

%%
%       ??
%    graph-based segmentation of the color image IM (HxWx3)
L = IGRAPHSEG(IM, K, MIN)
paper_fig2c__Zbuffer__rgb = ones(size(paper_fig2c__Zbuffer,1),size(paper_fig2c__Zbuffer,2),3,'uint8');
paper_fig2c__Zbuffer__rgb(:,:,1) = uint8(paper_fig2c__Zbuffer);
paper_fig2c__Zbuffer__rgb(:,:,2) = paper_fig2c__Zbuffer__rgb(:,:,1);
paper_fig2c__Zbuffer__rgb(:,:,3) = paper_fig2c__Zbuffer__rgb(:,:,1);
cd /mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/contrib/graphseg
mex -v graphseg.cpp
L = igraphseg(  paper_fig2c__Zbuffer__rgb , 1500 , 100 )
figure('Name','igraphseg'); idisp(L);

%%

iopen
ierode
idilate

idisp(255-double(BW1))
histogram(255-double(BW1))


BW3 = iconv(BW3,bilinear_kernel)   ; 
BW3 = iconv(BW3,bilinear_kernel)   ; 
idisp(BW3)
BW3 = BW3.* (255.0 / max(max(BW3))) -  (255.0 - double(BW1))  ;
idisp(BW3)
idisp(BW1 - (255.0 - BW1))
histogram(BW1 - (255.0 - BW1))


BW2 = bwmorph(BW1,'skel',inf);
figure('Name','skel - inf'), imshow(BW2)

BW2 = bwmorph(BW1,'skel',50);
figure('Name','skel - 50'), imshow(BW2)

BW2 = bwmorph(BW1,'skel',20);
figure('Name','skel - 20'), imshow(BW2)

BW2 = bwmorph(BW1,'skel',10);
figure('Name','skel - 10'), imshow(BW2)

BW2 = bwmorph(BW1,'thin',Inf);
figure('Name','thin - inf'), imshow(BW2)

BW2 = bwmorph(BW1,'thin',50);
figure('Name','thin - 50'), imshow(BW2)

BW2 = bwmorph(BW1,'thin',30);
figure('Name','thin - 30'), imshow(BW2)

% close all  