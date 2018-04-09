function display_map(map_with_layers, layers_to_display)
    if max(size(layers_to_display)) > 1    
        for ii_ = 1:max(size(layers_to_display))
            figure('Name',sprintf('Map layer %d',ii_));
            imshow(map_with_layers(:,:,ii_));
        end
    else
            ii_ = layers_to_display;
            figure('Name',sprintf('Map layer %d',ii_));
            imshow(map_with_layers(:,:,ii_));        
    end
end