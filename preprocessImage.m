function im = preprocessImage(I, doBlur, doNorm)
    % Convert to grayscale.
    gray = rgb2gray(I);
    
    % Blur.
    if doBlur
        G = fspecial('gaussian', 10, 5);
        im = imfilter(gray, G, 'symmetric');
    else
        im = gray;
    end
    
    if doNorm
        % Normalize pixel values.
        min_val = min(min(im));
        max_val = max(max(im));
        scale = 255/(max_val - min_val);
        im = (im - min_val)*scale;
    end
    
    % Convert to doubles.
    im = double(im);
end