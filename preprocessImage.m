function im = preprocessImage(I)
    % Convert to grayscale.
    gray = rgb2gray(I);
    
    % Blur.
    G = fspecial('gaussian', 10, 5);
    im = imfilter(gray, G, 'symmetric');
    
%     % Normalize pixel values.
%     min_val = min(min(im));
%     max_val = max(max(im));
%     scale = 255/(max_val - min_val);
%     im = (im - min_val)*scale;
    
    % Convert to doubles.
    im = double(im);
end