% Tracks pipe joints using a Kalman Filter and a Hough Transform to take
% measurements of the actual lines in the image.
function [new_line, weighted_norm, line] = edgeTracker(I, weights, max_num_skips, previous_line, evaluation)

    % Model parameters.
    A = [eye(3) eye(3); zeros(3) eye(3)];
    H = [eye(3) zeros(3)];

    % Uncertianty of the state (robot movement).
    Q = [0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 0 0 0;
         0 0 0 5 0 0;
         0 0 0 0 5 0;
         0 0 0 0 0 1];
    
    % Uncertainty of the measurement (line prediction).
    R = [10 0 0;
         0 10 0;
         0 0 5];
     
    % Measurement rejection Threshold
    error_threshold = 15;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Time Update (Prediction)          %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Estimate the new state based on model:  x_k = A * x_{k-1}
    state_prior = A * previous_line.state;

    % Estimate the covariance:  P_k = A * P_{k-1} * A^T + Q
    covariance_prior = A * previous_line.sigma * A' + Q;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Take a measurement                %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    if previous_line.real || previous_line.skip < max_num_skips+1
        [measurement, line] = trackLine(state_prior, covariance_prior, weights);
        found_line = ~isempty(measurement);
        
        % Pretend we got the predicted line as our measurement to keep the
        % covariance low.
        if ~found_line
            measurement = state_prior(1:3);
            previous_line.skip = previous_line.skip + 1;
        end
    else
        [measurement, line] = initializeLine();
        % HACK: this makes sure that we arent going in the direction of the
        % update to our measurement in the next step.
        if ~isempty(measurement)
            state_prior(1:3) = measurement;
%             if max_num_skips >= 2
%                 state_prior(4:6) = 0;
%             end
        end
        found_line = ~isempty(measurement);
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Outlier Rejection                 %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    weighted_norm = Inf;
    if ~isempty(measurement)
        S = H * covariance_prior * H' + R;
        error = measurement - H * state_prior;
        
        weighted_norm = (error'/S)*error;
        
        if weighted_norm > error_threshold
            measurement = [];
            found_line = false;
        end
        
        % If we are throwing out a line, but we are allowed to skip, use
        % the prior as our measurement to keep covariance from growing.
        if ~found_line
            measurement = state_prior(1:3);
            previous_line.skip = previous_line.skip + 1;
        end
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Measurement Update (Correction)   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    % If there is no measurement, do not do the update step.
    if isempty(measurement)
        % The posteriors are the same as the priors.
        state_posterior = state_prior;
        covariance_posterior = covariance_prior;        
    else
        % Determine the Kalman Gain:  K = P_k * H^T (H * P_k * H^T + R)^-1
        kalman_gain = covariance_prior * H' / (H * covariance_prior * H' + R);

        % Update the state measure:  x_k = x_k + K (z_k - H * x_k)
        state_posterior = state_prior + kalman_gain * (measurement - H * state_prior);

        % Update the covariance measure:  P_k = (I - K*H) P_k
        covariance_posterior = (eye(6) - kalman_gain * H) * covariance_prior;
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Create the return state     %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    new_line.state = state_posterior;
    new_line.sigma = covariance_posterior;
    new_line.real = found_line;
    new_line.skip = previous_line.skip;
   
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Visualize the found lines   %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if ~evaluation
        % Take square root of the covariance. We take simple square root.
        sigma = sqrt(covariance_posterior);
        
        % Show the image.
        imshow(uint8(I));
        hold on;
        % Plot the line.
        if ~isempty(line)
            xy = [line(1).point1; line(1).point2];
            plot(xy(:,1),xy(:,2), 'g', 'LineWidth',2);

            % Plot beginnings and ends of lines
            plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
            plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
        end
        
        % Plot the tracked state.
        vislines(new_line);
        
        drawnow;
    end
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    % Helper Functions      %
    %%%%%%%%%%%%%%%%%%%%%%%%%
       
    % Computes features for a new set of lines
    function f = getTrackingFeatures(I, new_lines, predicted_line)
        % Number of datapoints.
        n = size(new_lines, 1);
        
        %%%%%
        % MAGIC NUMBERS BELOW:
        %%%%%
        
        % Some small number.
        epsilon = 0.0001;
        % Approximate maximum that the distance measures can be.
        distance_max = 300;
        
        %%%%%
        % Feature Computation
        %%%%%

        % Compute the features for difference to predicted line.
        % This one does difference of the angle of the predicted line.
        % Compute change in angle.
        angle_delta = mod(new_lines(:, 3) - predicted_line(3), 360);
        angle_delta(angle_delta > 180) = 360 - angle_delta(angle_delta > 180);
        angle_score = (180 - angle_delta)./180;
        
        % Compute the features for difference to predicted line.
        % This one does difference from the center of the predicted line in
        % the x direction.
        x_diff = abs(new_lines(:, 1) - predicted_line(1));
        x_diff = (distance_max - x_diff)./distance_max;
        % If the we are further than the approximate max, give a low score.
        x_diff(x_diff <= 0) = epsilon;
                
        % This one does difference from the center of the predicted line in
        % the y direction.
        y_diff = abs(new_lines(:, 2) - predicted_line(2));
        y_diff = (distance_max - y_diff)./distance_max;
        % If the we are further than the approximate max, give a low score.
        y_diff(y_diff <= 0) = epsilon;
                
        % Compute the features.
        f = [angle_score x_diff y_diff];
        f = log(f);  % Compute the negative log of scores.
    end

    % Finds the lines in the image that are close to the prior.
    function [measurement, line] = trackLine(state_prior, covariance_prior, weights)
        % Use Hough Transform to find lines in the image.
        lines = imfindlines(I);
        
        % Put the lines into a usable struct.
        for i=1:length(lines)
            center = mean([lines(i).point1; lines(i).point2]);
            new_lines(i, :) = [center lines(i).theta lines(i).rho];
        end

        measurement = [];
        features = [];
        line = [];
        
        if ~isempty(lines)
            % Extract the features for each of the lines.
            features = getTrackingFeatures(I, new_lines, state_prior);

            % Determine the score from the learned weights.
            metric = features * weights;

            % Sort by the metric again.
            if ~isempty(new_lines)
                [~, ind] = sort(metric, 'descend');
                new_lines = new_lines(ind, :);
                features = features(ind, :);
                lines = lines(ind);
            end
            
            % The measurement is the best line.
            measurement = [new_lines(1, 1:3)]';
            line = lines(1);
        end
    end

    % Finds the strongest line in the image.
    function [measurement, line] = initializeLine()
        % Use Hough Transform to find lines in the image.
        lines = imfindlines(I);

        measurement = [];
        line = [];
        
        if ~isempty(lines)
            % Calculate the center of the best line.
            center = mean([lines(1).point1; lines(1).point2]);

            % TODO: what if the measurement is bad.

            % The measurement is the best line.
            measurement = [center lines(1).theta]';
            line = lines(1);
        end
    end
end