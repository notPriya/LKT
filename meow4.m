mov = VideoWriter('../../VO Presentation/bad_optical_flow2.avi');
mov.FrameRate = 10;
open(mov);

warp = getRigidBodyWarp();
for i=600:1000
    I = frames(:, :, :, i);
    M = LucasKanade(I, frames(:, :, :, i+1), eye(3), warp, [], [1 1 1 1]);
    
    [X Y] = warp.getXY(size(I));
    X = X(5:50:end, 5:50:end);
    Y = Y(5:50:end, 5:50:end);
    
    p1 = [X(:) Y(:)];
    p1 = [p1 ones(size(p1, 1), 1)]';
    
    p2 = M * p1;
    
    u = p1(1, :) - p2(1, :);
    v = p1(2, :) - p2(2, :);
    
    imshow(I);
    hold on;
    quiver(p1(1, :) + size(I, 2)/2, ...
           p1(2, :) + size(I, 1)/2, ...
           u, v, 'r');
    drawnow;
    hold off;
    
    writeVideo(mov, getframe);
    
    close;
end

close(mov);