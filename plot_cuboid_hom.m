function plot_cuboid_hom(vertices_hom_4x8, prefix, varargin) 
    
plot_cuboid(  cat(3,vertices_hom_4x8(:,1:4),vertices_hom_4x8(:,5:8)), vertices_hom_4x8, varargin{:})

if size(prefix,1)>0
    for ii_ = 1: 8    
        text( vertices_hom_4x8(1,ii_) , vertices_hom_4x8(2,ii_) , vertices_hom_4x8(3,ii_), strcat(prefix,'\_',int2str(ii_)), ...
            'Fontsize', 7, ...
            varargin{:})
    end
end