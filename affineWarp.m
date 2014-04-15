%% Affine warp used for LKT
function I2 = affineWarp(I, A)
    I2 = imwarp(I, affine2d(A'), 'OutputView', imref2d(size(I)), 'FillValues', NaN);
end