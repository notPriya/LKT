%% Affine warp used for LKT
function warp = getAffineWarp()
    warp.doWarp = @AffineWarp;
    warp.gradient = @getSpatialTensor;
    warp.compose = @AffineComposition;
    warp.newWarp = @createAffineWarp;
    warp.getXY = @getAffineCoordinates;
end

function I2 = AffineWarp(I, A)
    I2 = imwarp(I, affine2d(A'), 'OutputView', imref2d(size(I)), 'FillValues', NaN);
end

function Mfinal = AffineComposition(M, M_new)
    Mfinal = M_new * M;
end

function M = createAffineWarp(p)
    M = [1+p(1) p(2) p(3); p(4) 1+p(5) p(6); 0 0 1];
end

function A = getSpatialTensor(x, y, Ix, Iy)
    A = [x.*Ix, y.*Ix, Ix, x.*Iy, y.*Iy, Iy];
end

function [X, Y] = getAffineCoordinates(sizeI)
    [X, Y] = meshgrid(1:size(I, 2), 1:size(I, 1));
end