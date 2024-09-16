function [commMatrix] = generateCommunicationTopology(coords, A_ij, K)
    % Function to add communication links (as curves) to an existing physical topology

    epsilon = 10e-6;
    A = A_ij; % Adjacency matrix of the DG-DG communication topology
    % Number of Distributed Generators (DGs)
    numOfDGs = size(coords, 1);

    % Plot the existing physical topology
    figure; hold on;
    for i = 1:numOfDGs
        for j = i+1:numOfDGs
            if A(i, j) == 1
                % Coordinates of the physical link
                x = [coords(i,1) coords(j,1)];
                y = [coords(i,2) coords(j,2)];
                % Plot solid black line for physical link
                plot(x, y, 'k-', 'LineWidth', 2);
            end
        end
    end

    % Plot DGs as blue circles
    scatter(coords(:,1), coords(:,2), 100, 'bo', 'filled');
    title('Physical and Communication Topology');
    xlabel('X-coordinate');
    ylabel('Y-coordinate');
    grid on;

    % Initialize communication adjacency matrix
    commMatrix = zeros(numOfDGs, numOfDGs);

    % Add communication links as curves based on controller gains (K matrix)
    for i = 1:numOfDGs
        for j = i+1:numOfDGs
            % Check if the norm of the controller gain is greater than epsilon
            normVal = max(max(abs(K{i, j})));
            if normVal > epsilon
                % Update communication adjacency matrix
                commMatrix(i, j) = 1;
                commMatrix(j, i) = 1;

                % Coordinates of the communication link
                x1 = coords(i,1);
                x2 = coords(j,1);
                y1 = coords(i,2);
                y2 = coords(j,2);

                % Calculate midpoint and add an offset for curvature
                midX = (x1 + x2) / 2;
                midY = (y1 + y2) / 2;
                
                % Add a small offset perpendicular to the straight line to create curvature
                offset = 0.1;  % Adjust for more or less curve
                curveX = midX + offset * (y2 - y1);  % Perpendicular direction
                curveY = midY - offset * (x2 - x1);  % Perpendicular direction

                % Generate points for a smooth curve (quadratic curve)
                t = linspace(0, 1, 100);  % Parametric parameter
                curvePlotX = (1 - t).^2 * x1 + 2 * (1 - t) .* t * curveX + t.^2 * x2;
                curvePlotY = (1 - t).^2 * y1 + 2 * (1 - t) .* t * curveY + t.^2 * y2;

                % Plot the curved red dashed line for communication link
                plot(curvePlotX, curvePlotY, 'r--', 'LineWidth', 1.5);
            end
        end
    end

    hold off;
end
