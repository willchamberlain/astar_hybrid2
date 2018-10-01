%  see homography_calculate_and_apply

cursor_info_{1}=cursor_info
cursor_info_{2}=cursor_info
cursor_info_{end+1}=cursor_info
cursor_info_{end-1}.Position
cursor_info_{end}.Position

%%

figure_named('cursor_info_'); hold on; grid on; axis equal;
% pixels
PIN_pixels = zeros(size(cursor_info_,2),2) ;
for ii_ = 1:size(cursor_info_,2)
    plot2_rows(cursor_info_{ii_}.Position','rx')  ;
    text(cursor_info_{ii_}.Position(1),cursor_info_{ii_}.Position(2),int2str(ii_))  ;
    PIN_pixels(ii_,:) = cursor_info_{ii_}.Position  ;
end

%%

%{
PIN_pixels = [ ... % camera 2
   353   733   371   829   395   965
   610   551   716   637   879   756 ] ;
%}
fig_h = gcf  ;
axis_h = gca  ;
axis_h.YDir =  'reverse' ;

% pixels
if size(PIN_pixels,1) ~= 2
    PIN_pixels = PIN_pixels'
end
display( PIN_pixels )
figure; plot2_rows(PIN_pixels,'bx')
figure; plot3_rows(e2h(PIN_pixels),'bx'); hold on; grid on; axis equal; xlabel('x')

% world 
POUT_world = [   ...
    350,  500  ;   350,  -500 ;
    -650,  500  ;  -650,  -500 ;
   -1650,  500  ; -1650,  -500 ;
    ]'
figure; plot2_rows(POUT_world,'rx')

homography_Matrix_im_to_world = homography(PIN_pixels, POUT_world);  

%%  How to use:  

maybe_pin = inv(homography_Matrix_im_to_world) * e2h(POUT_world(:,1))  ;
maybe_pin = maybe_pin.*(1/maybe_pin(3))
PIN_pixels(:,1)

maybe_pout = homography_Matrix_im_to_world * e2h(PIN_pixels(:,1))  ;
maybe_pout = maybe_pout.*(1/maybe_pout(3))
POUT_world(:,1)



scale_from_m = 1000;
figure; imshow(frame);hold on;
for ii_ = 0:4
    maybe_pin = inv(homography_Matrix_im_to_world) * [ii_*scale_from_m;0;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'rx')
    maybe_pin = inv(homography_Matrix_im_to_world) * [ii_*scale_from_m;ii_*scale_from_m;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'bx')
    maybe_pin = inv(homography_Matrix_im_to_world) * [0;ii_*scale_from_m;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'gx')
    maybe_pin = inv(homography_Matrix_im_to_world) * [-ii_*scale_from_m;0;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'rs')
    maybe_pin = inv(homography_Matrix_im_to_world) * [-ii_*scale_from_m;ii_*scale_from_m;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'bs')
    maybe_pin = inv(homography_Matrix_im_to_world) * [0;ii_*scale_from_m;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'gs')
    maybe_pin = inv(homography_Matrix_im_to_world) * [-ii_*scale_from_m;0;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'rd')
    maybe_pin = inv(homography_Matrix_im_to_world) * [-ii_*scale_from_m;-ii_*scale_from_m;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'bd')
    maybe_pin = inv(homography_Matrix_im_to_world) * [0;-ii_*scale_from_m;1]  ;
    maybe_pin = maybe_pin.*(1/maybe_pin(3))
    hold on; plot(maybe_pin(1),maybe_pin(2),'gd')
end

%%


maybe_pin = inv(homography_Matrix_im_to_world) * [0;0;1]  ;
maybe_pin = maybe_pin.*(1/maybe_pin(3))


maybe_pos = homography_Matrix_im_to_world * [575;623;1]  ;
maybe_pos = maybe_pos.*(1/maybe_pos(3))
maybe_pos2 = homography_Matrix_im_to_world * [507;580;1]  ;
maybe_pos2 = maybe_pos2.*(1/maybe_pos2(3))
maybe_pos2  - maybe_pos
maybe_pos2 = homography_Matrix_im_to_world * [507;607;1]  ;
maybe_pos2 = maybe_pos2.*(1/maybe_pos2(3))
maybe_pos2  - maybe_pos

%%
class(homography_Matrix_im_to_world)
invhomog(homography_Matrix_im_to_world, 'K', eye(3)) 

cam_dummy = CentralCamera('resolution', [size(frame,2),size(frame,1)]   )
size(frame)
cam_dummy
cam_dummy.K
cam_dummy.invH(homography_Matrix_im_to_world)
cam_dummy.invH(inv(homography_Matrix_im_to_world))
[U,S,V] = svd(homography_Matrix_im_to_world)


%%   %%%
H = homography_Matrix_im_to_world

    % normalize H so that the second singular value is one
    [U,S,V] = svd(H);
    S
    H = H/S(2,2);
    H
    
    % compute the SVD of the symmetric matrix H'*H = VSV'
    [U,S,V] = svd(H'*H);
    
        
    % ensure V is right-handed
    if det(V) < 0,
        fprintf('det(V) was < 0\n');
        V = -V;
    end

    % get the squared singular values
    s1 = S(1,1);
    s3 = S(3,3);

    v1 = V(:,1); v2 = V(:,2); v3 = V(:,3);

    % pure the case of pure rotation all the singular values are equal to 1
    if abs(s1-s3) < 100*eps
        warning('Homoography due to pure rotation');
        if det(H) < 0
            H = -H;
        end
        sol(1).T = r2t(H);
        sol(1).n = [];
    else
        % compute orthogonal unit vectors
        u1 = (sqrt(1-s3)*v1 + sqrt(s1-1)*v3) / sqrt(s1-s3);
        u2 = (sqrt(1-s3)*v1 - sqrt(s1-1)*v3) / sqrt(s1-s3);
    %     disp('u1'); u1'
    %     disp('u2'); u2'

        U1 = [v2 u1 cross(v2,u1)];
        W1 = [H*v2 H*u1 skew(H*v2)*H*u1];

        U2 = [v2 u2 cross(v2,u2)];
        W2 = [H*v2 H*u2 skew(H*v2)*H*u2];

        % compute the rotation matrices
        R1 = W1*U1';
        R2 = W2*U2';

        % build the solutions, discard those with negative plane normals
        n = cross(v2, u1);
        if n(3) > 0,
            sol(1).n = n;
            t = (H-R1)*n;

        else
            sol(1).n = -n;
            t = -(H-R1)*n;
        end

        sol(1).T = inv([R1 t; 0 0 0 1]);

        n = cross(v2, u2);
        if n(3) > 0,
            sol(2).n = n;
            t = (H-R2)*n;
        else
            sol(2).n = -n;
            t = -(H-R2)*n;
        end
        sol(2).T = inv([R2 t; 0 0 0 1]);
    end
% 
%         for i=1:length(sol),
%             fprintf('\nsolution %d\n', i);
%             show('T =', sol(i).T)
%             show('n = ', sol(i).n')
% 
%             %T = sol(i).T;
%             %n = sol(i).n;
%             %H
%             %t2r(T)+transl(T)*n'
%             %det(t2r(T))
%             %inv(T)
%         end


