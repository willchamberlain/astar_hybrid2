function fig_handle_ = path_planning__draw_came_from(came_from_,figure_name_)
    fig_handle_ = figure('Name',figure_name_); 
    subplot(1,2,1); idisp(squeeze(came_from_(1,:,:)),'title','came_from(1)','here') ;  title('came\_from(1)') ;
    subplot(1,2,2); idisp(squeeze(came_from_(2,:,:)),'title','came_from(2)','here') ;  title('came\_from(2)') ;
    fig_handle_.set('Name',figure_name_);
end