

im_1 = imread('/home/will/Desktop/nice.png')  ;
im_2 = imread('/home/will/Desktop/nasty.png')  ;

block_size = [100,30]  ;
im_size = min( [ size(im_2) ; size(im_1) ] ) ;
block_size = [im_size(1),30]  ;
im_1 = im_1( 1:im_size(1) , 1:im_size(2) , 1:im_size(3))  ;
idisp(im_1)
im_2 = im_2( 1:im_size(1) , 1:im_size(2) , 1:im_size(3))  ;
idisp(im_2)
extra_pixels = mod(im_size(1:2),block_size)  
num_blocks = floor(im_size(1:2)./block_size)
num_blocks.*block_size
num_blocks.*block_size  +  extra_pixels 

im_1_orig = im_1 ;
im_2_orig = im_2 ;
for ii_=1:1000
    mix_ratio = 0.9;
    mix_ratio = min(max(abs(mod(ii_,100)/100 - 0.5)*2, 0.1),0.9)  ;
    im_1 = (im_1_orig*mix_ratio+im_2_orig*(1-mix_ratio))/2  ;
    im_2 = (im_1_orig*(1-mix_ratio)+im_2_orig*mix_ratio)/2  ;
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
    pause(0.03)
end


% for hh_ = 0:num_blocks(1)-1
%     for ww_ = 0:num_blocks(2)-1
%         mod( [hh_ ww_] , [2 2])
%         A(hh_+1,ww_+1)=sum(mod( [hh_ ww_] , [2 2])) ;
%         B = A==1 ;
%         block_bounds = [hh_ ww_; hh_+1 ww_+1] .* repmat(block_size, 2, 1)
%         if B(hh_+1,ww_+1)
%             im_to_use = im_1 ; 
%         else
%             im_to_use = im_2 ; 
%         end
%         %         idisp(im_to_use( block_bounds(1,1)+1:block_bounds(2,1) , block_bounds(1,2)+1:block_bounds(2,2) , : ))
%         %         pause(0.2)        
%         im_( block_bounds(1,1)+1:block_bounds(2,1) , block_bounds(1,2)+1:block_bounds(2,2) , : ) = (im_to_use( block_bounds(1,1)+1:block_bounds(2,1) , block_bounds(1,2)+1:block_bounds(2,2) , : )) ;
%     end
% end
% idisp(im_)