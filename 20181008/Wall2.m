classdef Wall2
    properties
        shape
        location
        location_anchor
    end
    methods
        function obj = Wall2(shape, location, location_anchor)
            obj.shape = shape  ;
            obj.location = location  ;
            obj.location_anchor = location_anchor  ;
        end
        
        function map__ = addToMap( obj, map_ )
            map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1) ...
                = map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1) + obj.shape;
            map__ = map_  ;
        end
        
        function map__ = mergeToMap( obj, map_ , varargin)
            if size(varargin) > 0
                if strcmpi(varargin{1}, 'max')
                    map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1) ...
                        = max( map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1)  ,  ...
                                obj.shape) ;
                elseif strcmpi(varargin{1}, 'min')
                    map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1) ...
                        = min( map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1)  ,  ...
                                obj.shape) ;
                end
            else
                    map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1) ...
                        = max( map_(obj.location(1):obj.location(1)+size(obj.shape,1)-1,obj.location(2):obj.location(2)+size(obj.shape,2)-1)  ,  ...
                                obj.shape) ;
            end
            map__ = map_  ;
        end
        
        function map__ = mergeToMapShapeoffset( obj, map_, varargin)            
            offset = [0 0] ;
            if size(varargin) > 0
                option_index = find(strcmpi(varargin,'offset'))  
                if ~isempty(option_index)
                    offset_type = varargin{option_index+1}  
                    if strcmpi(offset_type,'centre')
                        offset = round(size(obj.shape)./2) 
                    else
                        offset = [0 0] 
                    end
                end
                if any(strcmpi(varargin, 'max'))
                    display('max')
                    offset
                    offset(1)+obj.location(1)
                    offset(1)+obj.location(1)+size(obj.shape,1)-1
                    map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1) ...
                        = max( map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1)  ,  ...
                                obj.shape) ;
                elseif any(strcmpi(varargin, 'min'))
                    display('min')
                    offset
                    map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1) ...
                        = min( map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1)  ,  ...
                                obj.shape) ;
                elseif any(strcmpi(varargin, 'sum'))
                    display('sum')
                    offset
                    map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1) ...
                        =  map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1)   ...
                                +  obj.shape ;
                end                
            else
                offset
                display('no args')
                    map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1) ...
                        = max( map_(-offset(1)+obj.location(1):-offset(1)+obj.location(1)+size(obj.shape,1)-1,-offset(2)+obj.location(2):-offset(2)+obj.location(2)+size(obj.shape,2)-1)  ,  ...
                                obj.shape) ;
            end
            map__ = map_  ;
        end
        
    end
    
    
end 