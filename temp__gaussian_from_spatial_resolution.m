%%
%{

    Show the uncertainty of feature detection assuming 
        point feature / feature with points supporting it
        shallow arc at depth
        close to optical axis

%}



%%  Lateral:     Works pretty well - TODO: check deviation from Gaussian

p_= [] ;
for kk_ = 1:10:100
    for jj_ = 1:10:100
        [x,y] = meshgrid( -kk_:1:100-kk_  ,  -jj_:1:100-jj_  )  ;
        z=sqrt(x.^2+y.^2)  ;
        z_spread = max(max(z_vec)) - min(min(z_vec))  ;
        z_min = min(min(z_vec))  ;
        for ii_ = 1:10    
            %  uni_rand_10k = rand(1,101^2) .* max(max(z_vec)) ;
            uni_rand_10k = z_min + (  rand(1,101^2) .* z_spread  ) ;
            uni_rand_10k = reshape(uni_rand_10k, 1, 101^2)  ;
            p_ = [ p_  (uni_rand_10k - z_vec) ];
        end
    end
end
figure;  hist(p_, 100)

%%  Lateral:     Works pretty well - TODO: check deviation from Gaussian

p_= [] ;
for kk_ = 1:10:100
    for jj_ = 1:10:100
        [x,y] = meshgrid( -kk_:1:100-kk_  ,  -jj_:1:100-jj_  )  ;
        z=sqrt(x.^2+y.^2)  ;
        z_spread = max(max(z_vec)) - min(min(z_vec))  ;
        z_min = min(min(z_vec))  ;
        for ii_ = 1:10    
            uni_rand_10k = rand(1,101^2) .* max(max(z_vec)) ;
            % uni_rand_10k = z_min + (  rand(1,101^2) .* z_spread  ) ;
            % uni_rand_10k = reshape(uni_rand_10k, 1, 101^2)  ;
            p_ = [ p_  (uni_rand_10k - z_vec) ];
        end
    end
end
figure;  hist(p_, 100)

%%  Along optical axis:  if feature has extent, could be feature shrinks or_ grows so that some part of feature crosses into next pixel
%  Can be motion which increases or reduces distance. 
%  Feature dead-on optical axis will never move:  on the tail
%  Depends on definition of feature:  approximate as a point and/or the point on a feature ?most affected by a motion.

% Same sort of maths, but 
%  - motion is toward or away from the feature by uniformly-distributed distance 
%  - lateral change is due to divergence: due to the change sine of the angle of the point to the optical axis before and after the motion.
%  -- asymmetric in direction: toward will create larger divergence than away for the same distance :  approx as pixel small compared to distance therefore
%  similar triangles have bases nearly the same size
%  -- asymmetric across image:  greater parallax nearer edges of image
%
% --> magnitude of motion/distance change to produce pixel change > lateral motion/rotation change --> extended uncertainty along optical axis

uni_rand_10k = rand(1,101^2) - 0.5  ;
% figure;  plot(uni_rand_10k)








%%

uni_rand_20k = rand(1,20000)  ;
hist(uni_rand_20k)
hist(cumsum(uni_rand_20k))
plot(cumsum(uni_rand_20k))
[x,y] = meshgrid(1:101,1:101)  ;
[  size(x)  size(y)  ]
z=sqrt(x.^2+y.^2)  ;
surf(z)
min(min(z))
max(max(z))
size(z)
uni_rand_10k = rand(1,101^2)  ;
z_vec = reshape(z,1,101^2)  ;
max(max(z))
max(max(uni_rand_10k))
uni_rand_10k = rand(1,101^2) .* max(max(z_vec)) ;
plot(uni_rand_10k < z_vec)
hist(z_vec)
hist(uni_rand_10k)
plot(uni_rand_10k - z_vec)
hist(uni_rand_10k - z_vec)

%%


[x,y] = meshgrid(-50:1:50,-50:1:50)  ;
[ size(x)  size(y) ]
z=sqrt(x.^2+y.^2)  ;
surf(z)
min(min(z))
max(max(z))
size(z)
uni_rand_10k = rand(1,101^2)  ;
z_vec = reshape(z,1,101^2)  ;
max(max(z))
max(max(uni_rand_10k))
uni_rand_10k = rand(1,101^2) .* max(max(z_vec)) ;
plot(uni_rand_10k < z_vec)
hist(z_vec)
hist(uni_rand_10k)
plot(uni_rand_10k - z_vec)
hist(uni_rand_10k - z_vec)

%%


[x,y] = meshgrid(-25:1:75,-40:1:60)  ;
[ size(x)  size(y) ]
z=sqrt(x.^2+y.^2)  ;
surf(z)
min(min(z))
max(max(z))
size(z)
uni_rand_10k = rand(1,101^2)  ;
z_vec = reshape(z,1,101^2)  ;
max(max(z))
max(max(uni_rand_10k))
uni_rand_10k = rand(1,101^2) .* max(max(z_vec)) ;
plot(uni_rand_10k < z_vec)
hist(z_vec)
hist(uni_rand_10k)
plot(uni_rand_10k - z_vec)
hist(uni_rand_10k - z_vec)
p_= [] ;
for ii_ = 1:10    
    %  uni_rand_10k = rand(1,101^2) .* max(max(z_vec)) ;
    uni_rand_10k = rand(1,101^2) .* sqrt(101^2+101^2) ;
    p_ = [ p_  (uni_rand_10k - z_vec) ];
end
hist(p_, 100)

