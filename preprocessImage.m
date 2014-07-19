function im = preprocessImage(I, doBlur, doNorm, blur_params)
    if nargin < 4
        blur_params = [10 5];
    end

    % Convert to grayscale.
    gray = rgb2gray(I);
    
    % Blur.
    if doBlur
%         G = fspecial('gaussian', 5, 2);
        G = fspecial('gaussian', blur_params(1), blur_params(2));
        im = imfilter(gray, G, 'symmetric');
    else
        im = gray;
    end
    
    % Normalize Pixel Values.
    if doNorm
        % Normalize pixel values.
        min_val = min(min(im));
        max_val = max(max(im));
        scale = 255/(max_val - min_val);
        im = (im - min_val)*scale;
    end
    
%     % Downsample image.
%     x = 1:3:size(im, 1);
%     y = 1:3:size(im, 2);
%     im = im(x, y);

    % Make odd number of pixels.
    im = im(1:2*floor(size(im, 1)/2)-1, 1:2*floor(size(im, 2)/2)-1);
    
    % Convert to doubles.
    im = double(im);
end