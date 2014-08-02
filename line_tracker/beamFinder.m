% Finds the two most parallel lines in the image.
function edges = beamFinder(I)
    % Find all the lines in the image.
    lines = imfindlines(I, 'sobel');
    n = length(lines);
        
    if n < 1
        edges = [];
        return;
    end
    
    % Compare the theta's of each pair of observations.
    thetas = [lines.theta];
    dist = pdist(thetas');
    
    % Make it into a square symmetric matrix but then make all the
    % diagonals infinitely far away.
    dist = squareform(dist) + diag(Inf(1, n));
    
    % Find the ones with the smallest thetas.
    [vals, j] = min(dist);
    [score, i] = min(vals);
    j = j(i);
    
    % Thresholding the value for the anglular distance.
    if score > 5
        edges = [];
        return;
    end
    
    edges = [lines(i) lines(j)];
    
%     % Plotting code for reference.
%     imshow(uint8(I));
%     hold on;
%     for k=1:size(edges,2)
%         xy = [lines(k).point1; lines(k).point2];
%         if k==1
%             plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');
%         else
%             plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
%         end
% 
%         % Plot beginnings and ends of lines
%         plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%         plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
%     end
%     hold off;

end