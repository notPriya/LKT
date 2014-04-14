function [M, templateData] = LucasKanadeAffine(It, It1, M, templateData)
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
%         templateData.Ix = Ix;
%         templateData.Iy = Iy;
        templateData.A = [x.*Ix, y.*Ix, Ix, x.*Iy, y.*Iy, Iy];
    end
    
    % Compute the inititial parameters.
    V = [M(1,1)-1 M(1,2) M(1,3) M(2,1) M(2,2)-1 M(2,3)];    
    
    % Fix errors in M by computing small changes to the parameters.
    threshold = .1;
    iterations = 0;
    while (sum(abs(V)) > threshold || all(V==0)) && iterations < 10
        % Warp the image into the frame of the template.
        XiYi = M*[templateData.x templateData.y ones(length(templateData.x), 1)]';
        warpedI2 = interp2(1:size(I2, 2), 1:size(I2, 1), I2, XiYi(1, :)', XiYi(2, :)', 'linear', NaN);

        % Convert I to a column vector.
        warpedI = I(:);

        % Find NaN in the warped I2
        index = find(~isnan(warpedI2));
                
        % Drop all Nan's.
        x = templateData.x(index);
        y = templateData.y(index);
        warpedI2 = warpedI2(index);
        warpedI = warpedI(index);

        % Recompute Image gradient and Hessian with missing points.
        A = templateData.A(index, :);
        ATA = A'*A;
        
        % Compute image error.
        b = warpedI2 - warpedI;

        % Compute ATb
        ATb = A'*b;

        % Compute [a b c d e f]T
        V = ATA\ATb;

        % Compute the new change in warp.
        M1 = [1+V(1) V(2) V(3); V(4) 1+V(5) V(6); 0 0 1];
        
        % Update the existing warp.
        M = M / M1;
        
        % Count the number of iterations.
        iterations = iterations + 1;
    end
end