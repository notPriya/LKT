function [M, templateData, error] = LucasKanade(It, It1, M, warp, templateData)
    % Convert image to usable format.
    I = double(rgb2gray(It));
    I2 = double(rgb2gray(It1));
    
    if isempty(templateData)
        % Compute gradient
        [Ix, Iy] = gradient(I);

        % Convert Ix and Iy into column vectors
        Ix = Ix(:);
        Iy = Iy(:);

        % Compute X and Y
        [X, Y] = meshgrid(1:size(I, 2), 1:size(I, 1));
        x = X(:);
        y = Y(:);
        
        % Save off the template information into the template data struct.
        templateData.x = x;
        templateData.y = y;
        templateData.template = I(:);
        templateData.A = warp.gradient(x, y, Ix, Iy);
    end
    
    % Compute the inititial parameters.
    V = [1 1 1 1 1 1];    
    
    threshold = .00001;
    error = [];
    % Fix errors in M by computing small changes to the parameters.
    % Stop if we are not updating and the error is not still decreasing and
    % its not super high.
    while (sum(abs(V)) > threshold) && ...
          (     isempty(error) || ...
                (length(error) >= 2 &&  (error(end-1) - error(end)) > .001*error(end)) || ...
                (length(error) < 2 && error(end) > 10^4) ...
           )
        % Warp the image into the frame of the template.
        warpedI2 = warp.doWarp(I2, M);
        warpedI2 = warpedI2(:);
        
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

        % Compute ATb
        ATb = A'*b;

        % Compute [a b c d e f]T
        V = ATA\ATb;

        % Compute the new change in warp.
        M1 = warp.newWarp(V);
        
        % Update the existing warp.
        M = warp.compose(M, M1);        
    end
end