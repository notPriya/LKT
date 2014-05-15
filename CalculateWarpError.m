% Calculates error of an image from a template image for a warp that is
% non-linear in the parameters. Useful for non-linear optimizations.
function error = CalculateWarpError(p, It, It1, warp, odom_rect)
    % Fuck nans. =C
    if any(isnan(p))
        disp('Got nan for p');
        error = Inf;  % Over 9000.
        return
    end
    
    % Convert the current parameters into a warp.
    M = warp.newWarp(p);
    
    % Convert image to usable format.
    I = preprocessImage(It);
    I2 = preprocessImage(It1);
                
    % Create a mask for the odometry rectangle.
    mask = zeros(size(I2));
    mask(odom_rect(3):odom_rect(4), odom_rect(1):odom_rect(2)) = 1;
    
    % Save off the template information.
    template = I(:);

    % Warp the image into the frame of the template.
    warpedI2 = warp.doWarp(I2, M);
    warpedI2 = warpedI2(:);
        
    % Warp the odometry mask.
    warpedMask = warp.doWarp(mask, M);
    warpedMask(isnan(warpedMask)) = 0;
    warpedMask = logical(warpedMask(:));
        
    % Remove the odometry rectangle from the warp.
    warpedI2(warpedMask) = NaN;
        
    % Find NaN in the warped I2
    index = find(~isnan(warpedI2));
                
    % Drop all Nan's.
    warpedI2 = warpedI2(index);
    warpedTemplate = template(index);

    % Compute image error.
    b = warpedI2 - warpedTemplate;
    
    % Error is weighted average between average pixel error and penalty for too many NaNs.
    average_pixel_error = mean(abs(b));
    error = average_pixel_error;
end