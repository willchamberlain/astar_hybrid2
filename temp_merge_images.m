

im_1 = imread('/home/will/Desktop/nice.png')  ;
im_2 = imread('/home/will/Desktop/nasty.png')  ;

block_size = [100,30]  ;
im_size = min( [ size(im_2) ; size(im_1) ] ) ;
im_1 = im_1( 1:im_size(1) , 1:im_size(2) , 1:im_size(3))  ;
idisp(im_1)
im_2 = im_2( 1:im_size(1) , 1:im_size(2) , 1:im_size(3))  ;
idisp(im_2)
extra_pixels = mod(im_size(1:2),block_size)  
num_blocks = floor(im_size(1:2)./block_size)
num_blocks.*block_size
num_blocks.*block_size  +  extra_pixels 


for hh_ = 0:num_blocks(1)-1
    for ww_ = 0:num_blocks(2)-1
        mod( [hh_ ww_] , [2 2])
        A(hh_+1,ww_+1)=sum(mod( [hh_ ww_] , [2 2])) ;
        B = A==1 ;
        block_bounds = [hh_ ww_; hh_+1 ww_+1] .* repmat(block_size, 2, 1)
        if B(hh_+1,ww_+1)
            im_to_use = im_1 ; 
        else
            im_to_use = im_2 ; 
        end
        %         idisp(im_to_use( block_bounds(1,1)+1:block_bounds(2,1) , block_bounds(1,2)+1:block_bounds(2,2) , : ))
        %         pause(0.2)        
        im_( block_bounds(1,1)+1:block_bounds(2,1) , block_bounds(1,2)+1:block_bounds(2,2) , : ) = (im_to_use( block_bounds(1,1)+1:block_bounds(2,1) , block_bounds(1,2)+1:block_bounds(2,2) , : )) ;
    end
end
idisp(im_)

