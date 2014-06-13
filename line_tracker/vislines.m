function vislines(line)
    % Compute the endpoints of the line.
    theta = line.state(3);
    point1 = line.state(1:2) + 150*[-sind(theta); cosd(theta)];
    point2 = line.state(1:2) - 150*[-sind(theta); cosd(theta)];

    % Plot the line.
    xy = [point1'; point2'];
    if line.real
        plot(xy(:,1),xy(:,2), 'b', 'LineWidth',2);
    else
        plot(xy(:,1),xy(:,2), 'b--', 'LineWidth',2);
    end

    % Plot beginnings and ends of lines
    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
end