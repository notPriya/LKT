function [M, templateData, error] = LucasKanade(It, It1, M, warp, templateData, odom_rect)
    % Convert image to usable format.
    I2 = preprocessImage(It1);
    
    
    if isempty(templateData)
        I = preprocessImage(It);
        
        % Compute gradient
        [Ix, Iy] = gradient(I);

        % Convert Ix and Iy into column vectors
        Ix = Ix(:);
        Iy = Iy(:);

        % Compute X and Y
        [X, Y] = warp.getXY(size(I));
        x = X(:);
        y = Y(:);
        
        % Create a mask for the odometry rectangle.
        mask = zeros(size(I));
        mask(odom_rect(3):odom_rect(4), odom_rect(1):odom_rect(2)) = 1;
    
        % Save off the template information into the template data struct.
        templateData.x = x;
        templateData.y = y;
        templateData.template = I(:);
        templateData.A = warp.gradient(x, y, Ix, Iy);
        templateData.mask = mask;
    end
    
    % Compute the inititial parameters.
    V = [1; 1; 1; 1];    
    
    threshold = .00001;
    error = [];
    max_iteration = 30;
    Ms = zeros(3, 3, max_iteration);
    iteration = 1;
    % Fix errors in M by computing small changes to the parameters.
    % Stop if we are not updating and the error is not still decreasing or
    % if we have maxed the number of iterations and the error hasnt gone
    % down enough.
    while ((sum(abs(V)) > threshold) && ...
          iteration < max_iteration && ...
          (length(error) < 5 || error(end-1) - error(end) > 1000)) || ...  %.001*error(end))) || ...
          ((isempty(error) || error(end) > 5*10^6) && ...
          iteration < max_iteration)
        % Warp the image into the frame of the template.
        warpedI2 = warp.doWarp(I2, M);
        warpedI2 = warpedI2(:);
        
        % Warp the odometry mask.
        warpedMask = warp.doWarp(templateData.mask, M);
        warpedMask(isnan(warpedMask)) = 0;
        warpedMask = logical(warpedMask(:));
        
        % Remove the odometry rectangle from the warp.
        warpedI2(warpedMask) = NaN;  % The warped mask for the warped image odometry box.
        warpedI2(logical(templateData.mask(:))) = NaN;  % The unwarped mask for the template odometry box.
        
        % Find NaN in the warped I2
        index = find(~isnan(warpedI2));
                
        % Drop all Nan's.
        warpedI2 = warpedI2(index);
        warpedTemplate = templateData.template(index);

        % Recompute Image gradient and Hessian with missing points.
        A = templateData.A(index, :);
        ATA = A'*A;
        
        % Compute image error.
        b = warpedI2 - warpedTemplate;
        error = [error; sum(abs(b))];
        Ms(:, :, iteration) = M;

        % Compute ATb
        ATb = A'*b;

        % Compute [a b c d e f]T
        V = ATA\ATb;

        % Compute the new change in warp.
        M1 = warp.newWarp(V);
        
        % Update the existing warp.
        M = warp.compose(M, M1);
        
        iteration = iteration+1;
    end
    
    [~, index] = min(error);
    M = Ms(:, :, index);
end