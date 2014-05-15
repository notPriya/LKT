%% Nonlinear Rigid Body warp used for LK-nonlinear
function warp = getNonLinWarp()
    warp.doWarp = @nonLinearWarp;
    warp.gradient = @getSpatialTensor;
    warp.compose = @nonLinearComposition;
    warp.newWarp = @createNonLinearTransform;
    warp.getParam = @getNonLinearParameters;
end

function I2 = nonLinearWarp(I, T)
    % Put (0, 0) in the center of the image.
    RI = imref2d(size(I), [-size(I, 2)/2 size(I, 2)/2], [-size(I, 1)/2 size(I, 1)/2]);    
    I2 = imwarp(I, RI, affine2d(T'), 'OutputView', RI, 'FillValues', NaN);
end

function Tfinal = nonLinearComposition(T, Tnew)
    Tfinal = T * Tnew;
end

function T = createNonLinearTransform(p)
%     T = [1+p(3) -p(4) p(1); p(4) 1+p(3) p(2); 0 0 1];
    T = p(3) .* [1 -p(4) p(1); p(4) 1 p(2); 0 0 1];
    T(3, :) = [0 0 1];
end

function A = getSpatialTensor(x, y, Ix, Iy)
    A = [Ix Iy x.*Ix+y.*Iy x.*Iy-y.*Ix];
end

function p = getNonLinearParameters(M)
    alpha = M(1,1); 
    p = 1/alpha .* [M(1,3); M(2, 3); M(1, 1); M(2, 1)];
    p(3) = alpha;
end