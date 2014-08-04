% Uses a Hough transform to find lines in the image.
function lines = imfindlines(I, edge_detector)    
    % If the edge detection algorithm is not passed in, default to canny
    % edge detection.
    if nargin < 2
        edge_detector = 'canny';
    end
    
    % Find the edges in the image using the Canny detector.
    BW = edge(I, edge_detector);

    % Compute the Hough transform of the image.
    [H,theta,rho] = hough(BW);
    
    % Find the peaks in the Hough transform matrix, H.
    P = houghpeaks(H, 40,'threshold',ceil(0.3*max(H(:))));
    
    % Find lines in the image using the houghlines function.
    lines = houghlines(BW,theta,rho,P,'FillGap',40,'MinLength', 300);

%     %% Plotting code for reference.
%     I(BW) = 0;
%     imshow(uint8(I));
%     hold on;
%     for k=1:size(lines,2)
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