%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Initialize the environment  %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc;

% Determine which video to use.
if ~exist('pipe_name', 'var')	
	pipe_name = input('Which crawler video should we use?   ', 's');
end

% Load the frames to process.
if ~exist('frames', 'var')
    load([pipe_name '.mat']);
end

% Initialize frame variables.
start = 1;
n = size(frames, 4) - start;

% Plotting stuff.
evaluation = true;

% Prior on the beam widths.
beam_width = 0.0508;  %% 2 inches for half a beam.

% Scale factor to go from pixels to real world units.
scale_factor = 1;
camera_f = 510;

% Estimated position of the robot.
estimates = zeros(n, 1);

% Least squares matrix.
A = [];
b = [];

% Run the beam finder.
for i=start:start+n
    % Get the preprocessed image.
    I = preprocessImage(frames(:,:,:,i), true, false, [5 2]);
    I = rot90(I, -1);

    % Look for parallel lines in the image.
    beam = beamFinder(I);
    
    % If we found a beam, re-compute the least squares solution.
    if ~isempty(beam)
        % Calculate the slope, intercept form of the lines.
        m1 = (beam(1).point2(2) - beam(1).point1(2))/(beam(1).point2(1)-beam(1).point1(1));
        m2 = (beam(2).point2(2) - beam(2).point1(2))/(beam(2).point2(1)-beam(2).point1(1));
        b1 = beam(1).point1(2) - m1*beam(1).point1(1);
        b2 = beam(2).point1(2) - m2*beam(2).point1(1);
        
        % Compute the distance between the beam in the image.
        pix_width = abs(b2 - b1)/sqrt(m1*m2+1);
        
        % Add it to the least squares matrix.
        A = [A; pix_width];
        b = [b; beam_width];

        % Solve the least squares solution.
        scale_factor = A\b;
    end
    
    % Save off the scale factor as the current estimate.
    estimates(i) = scale_factor;
    
    % Visualize the lines.
    if ~evaluation
        imshow(uint8(I));
        hold on;
        for k=1:size(beam,2)
            xy = [beam(k).point1; beam(k).point2];
            if k==1
                plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','blue');
            else
                plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
            end

            % Plot beginnings and ends of lines
            plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        end
        drawnow;
        hold off;
    end

end


