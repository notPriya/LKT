%% Rigid Body warp used for LKT
function warp = getRigidBodyWarp()
    warp.doWarp = @rigidBodyWarp;
    warp.gradient = @getSpatialTensor;
    warp.compose = @rigidBodyComposition;
    warp.newWarp = @createRigidBodyTransform;
    warp.getXY = @getRigidBodyCoordinates;
end

function I2 = rigidBodyWarp(I, T)
    % Put (0, 0) in the center of the image.
    RI = imref2d(size(I), [-size(I, 2)/2 size(I, 2)/2], [-size(I, 1)/2 size(I, 1)/2]);    
    I2 = imwarp(I, RI, affine2d(T'), 'OutputView', RI, 'FillValues', NaN);
end

function Tfinal = rigidBodyComposition(T, Tnew)
    Tfinal = T * Tnew;
end

function T = createRigidBodyTransform(p)
    T = [1+p(3) -p(4) p(1); p(4) 1+p(3) p(2); 0 0 1];
end

function A = getSpatialTensor(x, y, Ix, Iy)
    A = [Ix Iy x.*Ix+y.*Iy x.*Iy-y.*Ix];
end

function [X, Y] = getRigidBodyCoordinates(sizeI)
    [X, Y] = meshgrid(ceil(-sizeI(2)/2):floor(sizeI(2)/2), ...
                      ceil(-sizeI(1)/2):floor(sizeI(1)/2));
end