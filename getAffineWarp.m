%% Affine warp used for LKT
function warp = getAffineWarp()
    warp.doWarp = @AffineWarp;
    warp.gradient = @getSpatialTensor;
    warp.compose = @AffineComposition;
    warp.newWarp = @createAffineWarp;
    warp.getXY = @getAffineCoordinates;
end

function I2 = AffineWarp(I, A)
    % Put (0, 0) in the center of the image.
    RI = imref2d(size(I), [-size(I, 2)/2 size(I, 2)/2], [-size(I, 1)/2 size(I, 1)/2]);    
    I2 = imwarp(I, RI, affine2d(A'), 'OutputView', RI, 'FillValues', NaN);
end

function Mfinal = AffineComposition(M, M_new)
    Mfinal = M * M_new;
end

function M = createAffineWarp(p)
    M = [1+p(1) p(2) p(3); p(4) 1+p(5) p(6); 0 0 1];
end

function A = getSpatialTensor(x, y, Ix, Iy)
    A = [x.*Ix, y.*Ix, Ix, x.*Iy, y.*Iy, Iy];
end

function [X, Y] = getAffineCoordinates(sizeI)
    [X, Y] = meshgrid(ceil(-sizeI(2)/2):floor(sizeI(2)/2), ...
                      ceil(-sizeI(1)/2):floor(sizeI(1)/2));
end