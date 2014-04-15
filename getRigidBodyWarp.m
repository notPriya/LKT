%% Rigid Body warp used for LKT
function warp = getRigidBodyWarp()
    warp.doWarp = @rigidBodyWarp;
    warp.gradient = @getSpatialTensor;
    warp.compose = @rigidBodyComposition;
    warp.newWarp = @createRigidBodyTransform;
end

function I2 = rigidBodyWarp(I, T)
    I2 = imwarp(I, affine2d(T'), 'OutputView', imref2d(size(I)), 'FillValues', NaN);
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