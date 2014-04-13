function [u, v, templateData] = LucasKanade(It, It1, rect, templateData)
    % Parameters of the LK traker.
    threshold = 0.05;  % Threshold of convergence.
    
    % Initialize variables.
    u = 0;
    v = 0;
    V = [1, 1];
    [xi, yi] = meshgrid(rect(1):rect(3), rect(2):rect(4));
    
    % If we havent computed information about the template yet, do so.
    % Then save all the information so we arent computing it over and over.
    if isempty(templateData)
        % Get the grayscale image.
        I = double(rgb2gray(It));

        % Interpolate the template.
        [x, y] = meshgrid(1:size(It, 2), 1:size(It, 1));
        template = interp2(x, y, I, xi, yi);
        template = double(template);

        % Compute gradient
        [Ix, Iy] = gradient(template);

        % Convert Ix and Iy into column vectors
        Ix = Ix(:);
        Iy = Iy(:);

        % Compute ATA (Hessian?)
        Ixx = sum(Ix.^2);
        Ixy = sum(Ix.*Iy);
        Iyy = sum(Iy.^2);
        
        % Save off the template information into the template data struct.
        templateData.x = x;
        templateData.y = y;
        templateData.template = template;
        templateData.Ix = Ix;
        templateData.Iy = Iy;
        templateData.Hessian = [Ixx Ixy; Ixy Iyy];
    end
    
    % Warp the current image to fit the template.
    while(norm(V) > threshold)        
%         % Visualize
%         imshow(It1);
%         rectangle('Position', [rect(1), rect(2), rect(3)-rect(1), rect(4)-rect(2)]);
%         pause(.5);
        
        % Interpolate the new template.
        image = double(rgb2gray(It1));
        template_prime = interp2(templateData.x, templateData.y, image, xi+u, yi+v);
        
        % Compute It (except something already is It, so we've
        % called this variable b).
        b = templateData.template - template_prime;
        b = b(:);
        b = double(b);

        % Compute ATb
        IxIt = templateData.Ix'*b;
        IyIt = templateData.Iy'*b;

        % Compute [u v]T
        V = templateData.Hessian\[IxIt; IyIt];

        % Extract u and v
        u = u + V(1);
        v = v + V(2);
        
        % Update the window
        rect = rect + [V(1) V(2) V(1) V(2)];
    end

end