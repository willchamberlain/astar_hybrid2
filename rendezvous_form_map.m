%  Developed in  /mnt/nixbig/ownCloud/project_code/temp__vrep_drive_pioneer_2018_10_16.m


%-       load floorplan 
%{
    /mnt/nixbig/build_workspaces/aa1_vos_android_catkin_ws/src/vos_aa1/launch/map_server_2.launch
        /mnt/nixbig/data/project_AA1_backup_pioneer3c/lvl11_map2/map2_dummy.yaml 
        use
        /mnt/nixbig/data/project_AA1_backup_pioneer3c/lvl11_map2/map.pgm
        /mnt/nixbig/data/project_AA1_backup_pioneer3c/lvl11_map2/map.yaml
            NOT going to bother with a YAML parser at the moment
                image: map.pgm
                resolution: 0.050000
                origin: [-100.000000, -100.000000, 0.000000]
                negate: 0
                occupied_thresh: 0.65
                free_thresh: 0.196    
    %}
resolution = 0.050000  ;

    idisp(map_image>=210)

floor_space = map_image>=210  ;
map_image_2 = iopen(floor_space, ones(3),3)  ;  % remove the noise

    idisp(map_image_2 )

% use bwdist to keep centre of FoV away from walls and small spaces
[D,IDX] = bwdist(1-map_image_2) ;
    idisp(D)

% this is the one I visualise with :   map_image_good_spots(x,y)
map_image_good_spots = D<5*(1/resolution) & D>0.6*(1/resolution)   ;

% this is the one I look up with :   map_image_good_spots_values(x,y)
map_image_good_spots_values = map_image_good_spots' ;

idisp( map_image_good_spots )  ; hold on


