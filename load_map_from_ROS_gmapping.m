function map__ = load_map_from_ROS_gmapping(varargin)
    %%  ---------------------------  start loading the map from a map used for ROS
    %-       load floorplan 
    %{
        /mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/launch/map_server_2.launch
            map.yaml and map.pgm  in  /mnt/nixbig/data/project_AA1_backup_pioneer3c/lvl11_map2/
                NOT going to bother with a YAML parser at the moment
                    image: map.pgm
                    resolution: 0.050000
                    origin: [-100.000000, -100.000000, 0.000000]
                    negate: 0
                    occupied_thresh: 0.65
                    free_thresh: 0.196    
        %}

    p = inputParser;
    % add optional needs the name, default value, and validator
    p.addOptional('filepathname','/mnt/nixbig/data/project_AA1_backup_pioneer3c/lvl11_map2/map_small.pgm',@isstr);
    % add optional needs the name, default value, and validator
    p.addOptional('resolution_m_per_cell',0.050000,@isfloat);

    % p

    p.parse(varargin{:})
    options_ = p.Results;

    % options_

    resolution =  options_.resolution_m_per_cell ;  
    map_image = imread(options_.filepathname)  ;
    % idisp(map_image)
    % display(  size(map_image)   ) ;  display(  class(map_image) )  ;
    % [n,edges,bin] = histcounts(map_image,256);

    %-- remove noise and find regions away from the walls/obstacles  
    % idisp(map_image>=210)
    floor_space = map_image>=210  ;
    map_image_2 = iopen(floor_space, ones(3),3)  ;  % remove the noise
    % idisp(map_image_2 )
    map__ = map_image_2 ;
end